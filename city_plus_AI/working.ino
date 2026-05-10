/*
 * ================================================================
 *  WATER JET LOOM EMULATOR v3.0 — ESP32 FIRMWARE
 *  CityPlus AI — Industrial Predictive Maintenance
 *  NMIT Hacks 2026
 * ================================================================
 *
 *  HARDWARE WIRING
 *  Switch 1 (Thread Break)         → GPIO 18 + GND
 *  Switch 2 (Pressure Instability) → GPIO 19 + GND
 *  Switch 3 (Pump Degradation)     → GPIO 21 + GND
 *  Buzzer (+)                      → GPIO 23 + GND
 *  All switches: INPUT_PULLUP — press = LOW
 *
 *  MQTT CONTROL COMMANDS (topic: textile/machine/WJ_01/control)
 *  "STOP"        → pause publishing (machine shows offline)
 *  "START"       → resume publishing
 *  "RESET_FAULT" → clear fault, return to normal
 *
 *  LIBRARIES REQUIRED
 *  PubSubClient by Nick O'Leary
 *  ArduinoJson  by Benoit Blanchon v6.x
 *
 *  Board: ESP32 Dev Module | Baud: 115200
 * ================================================================
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

// ================================================================
//  ★ EDIT THESE THREE BEFORE UPLOADING ★
// ================================================================
const char* WIFI_SSID          = "OnePlus 11R 5G";
const char* WIFI_PASSWORD      = "zaveriya";
const char* MQTT_BROKER        = "10.174.5.100";
// ================================================================

const int   MQTT_PORT          = 1883;
const char* MACHINE_ID         = "WJ_01";
const char* MQTT_DATA_TOPIC    = "textile/machine/WJ_01/data";
const char* MQTT_CONTROL_TOPIC = "textile/machine/WJ_01/control";

// ── GPIO ─────────────────────────────────────────────────────────
const int SW_THREAD_BREAK    = 18;
const int SW_PRESSURE_INSTAB = 19;
const int SW_PUMP_DEGRAD     = 21;
const int BUZZER_PIN         = 23;

const unsigned long PUBLISH_MS = 2000;

// ── FAULT ENUM ───────────────────────────────────────────────────
enum FaultType {
  FAULT_NONE                 = 0,
  FAULT_THREAD_BREAK         = 1,
  FAULT_PRESSURE_INSTABILITY = 2,
  FAULT_PUMP_DEGRADATION     = 3
};

// ================================================================
//  MACHINE STATE
//  NOTE: status and fault_code are Arduino String objects.
//        This allows == comparisons with string literals.
// ================================================================
struct MachineState {
  float  temperature_c;
  float  motor_load_percent;
  float  pump_pressure_bar;
  float  nozzle_pressure_bar;
  float  water_flow_lpm;
  float  vibration_x;
  float  vibration_y;
  float  vibration_z;
  float  runtime_hours;
  float  health_score;

  float  target_temp;
  float  target_load;
  float  target_pressure;
  float  target_flow;
  float  target_vib;

  FaultType fault_type;
  String    fault_code;   // ← String, NOT const char*
  String    status;       // ← String, NOT const char*
  int       phase;
};

MachineState machine;

// ── CONTROL FLAGS ─────────────────────────────────────────────────
bool publishingEnabled = true;
bool manualFaultActive = false;

int lastSw1 = HIGH;
int lastSw2 = HIGH;
int lastSw3 = HIGH;

unsigned long lastPublishTime = 0;
unsigned long startMillis     = 0;

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// ================================================================
//  HELPERS
// ================================================================
float rFloat(float lo, float hi) {
  return lo + ((float)random(0, 10000) / 10000.0f) * (hi - lo);
}

float clampF(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

float lerpToward(float cur, float tgt, float factor, float noise) {
  return cur + (tgt - cur) * factor + rFloat(-noise, noise);
}

// ================================================================
//  TIMESTAMP
// ================================================================
String makeTimestamp() {
  unsigned long base    = 1746662400UL;
  unsigned long elapsed = (millis() - startMillis) / 1000UL;
  unsigned long epoch   = base + elapsed;

  unsigned long ss  = epoch % 60;
  unsigned long mm  = (epoch / 60) % 60;
  unsigned long hh  = (epoch / 3600) % 24;
  unsigned long days = epoch / 86400UL;

  unsigned long y = 1970, dc = 0;
  while (true) {
    bool leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
    unsigned long diy = leap ? 366 : 365;
    if (dc + diy > days) break;
    dc += diy; y++;
  }
  unsigned long doy = days - dc;
  int md[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) md[1] = 29;
  int mon = 1;
  unsigned long d = doy;
  for (int i = 0; i < 12; i++) {
    if (d < (unsigned long)md[i]) { mon = i + 1; d++; break; }
    d -= md[i];
  }
  char buf[25];
  snprintf(buf, sizeof(buf), "%04lu-%02d-%02luT%02lu:%02lu:%02luZ",
           y, mon, d, hh, mm, ss);
  return String(buf);
}

// ================================================================
//  WIFI
// ================================================================
void connectWiFi() {
  Serial.println("\n[WiFi] Connecting to: " + String(WIFI_SSID));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
    if (++tries > 40) {
      Serial.println("\n[WiFi] Timeout — retrying...");
      delay(3000);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      tries = 0;
    }
  }
  Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());
}

// ================================================================
//  MQTT CALLBACK
// ================================================================
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg = "";
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  Serial.println("[CTRL] Received: " + msg);

  if (msg == "STOP") {
    publishingEnabled = false;
    Serial.println("[CTRL] Publishing STOPPED");

  } else if (msg == "START") {
    publishingEnabled = true;
    Serial.println("[CTRL] Publishing RESUMED");

  } else if (msg == "RESET_FAULT") {
    manualFaultActive  = false;
    machine.fault_type = FAULT_NONE;
    machine.fault_code = "normal";
    machine.status     = "running";
    machine.phase      = 0;
    machine.target_temp     = rFloat(42.0, 52.0);
    machine.target_load     = rFloat(55.0, 78.0);
    machine.target_pressure = rFloat(100.0, 130.0);
    machine.target_flow     = rFloat(22.0, 30.0);
    machine.target_vib      = rFloat(0.12, 0.35);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("[CTRL] Fault CLEARED — returning to normal");
  }
}

// ================================================================
//  MQTT CONNECT
// ================================================================
void connectMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setBufferSize(640);
  mqttClient.setCallback(mqttCallback);
  while (!mqttClient.connected()) {
    String cid = "WJ01_" + String(random(0xffff), HEX);
    Serial.println("[MQTT] Connecting as " + cid);
    if (mqttClient.connect(cid.c_str())) {
      Serial.println("[MQTT] Connected");
      mqttClient.subscribe(MQTT_CONTROL_TOPIC);
    } else {
      Serial.println("[MQTT] Failed rc=" + String(mqttClient.state()) + " — retry 5s");
      delay(5000);
    }
  }
}

// ================================================================
//  FAULT TARGET SETTERS
// ================================================================
void setNormalTargets() {
  machine.target_temp     = rFloat(42.0,  52.0);
  machine.target_load     = rFloat(55.0,  78.0);
  machine.target_pressure = rFloat(100.0, 130.0);
  machine.target_flow     = rFloat(22.0,  30.0);
  machine.target_vib      = rFloat(0.12,  0.35);
  machine.fault_type      = FAULT_NONE;
  machine.fault_code      = "normal";
  machine.status          = "running";
  machine.phase           = 0;
}

void setThreadBreakTargets() {
  machine.target_temp     = rFloat(44.0,  53.0);
  machine.target_load     = rFloat(22.0,  35.0);   // tension lost
  machine.target_pressure = rFloat(125.0, 148.0);  // brief spike
  machine.target_flow     = rFloat(14.0,  18.0);   // flow drops
  machine.target_vib      = rFloat(0.30,  0.60);
  machine.fault_type      = FAULT_THREAD_BREAK;
  machine.fault_code      = "thread_break";
  machine.status          = "fault";
  machine.phase           = 2;
}

void setPressureInstabilityTargets() {
  machine.target_temp     = rFloat(44.0,  54.0);
  machine.target_load     = rFloat(62.0,  82.0);
  machine.target_pressure = rFloat(55.0,  155.0);  // wild swing
  machine.target_flow     = rFloat(18.0,  32.0);   // unstable
  machine.target_vib      = rFloat(0.50,  1.20);   // all axes rise
  machine.fault_type      = FAULT_PRESSURE_INSTABILITY;
  machine.fault_code      = "pressure_instability";
  machine.status          = "warning";
  machine.phase           = 1;
}

void setPumpDegradationTargets() {
  machine.target_temp     = rFloat(48.0,  62.0);
  machine.target_load     = rFloat(72.0,  94.0);
  machine.target_pressure = rFloat(62.0,  78.0);   // low pressure
  machine.target_flow     = rFloat(16.0,  22.0);
  machine.target_vib      = rFloat(1.00,  1.80);   // high vibration
  machine.fault_type      = FAULT_PUMP_DEGRADATION;
  machine.fault_code      = "pump_degradation";
  machine.status          = "fault";
  machine.phase           = 2;
}

// ================================================================
//  HEALTH SCORE
// ================================================================
float computeHealthScore() {
  float score = 100.0;

  if (machine.temperature_c > 52.0)
    score -= (machine.temperature_c - 52.0f) * 1.8f;
  if (machine.temperature_c < 38.0)
    score -= (38.0f - machine.temperature_c) * 1.5f;

  if (machine.pump_pressure_bar < 100.0)
    score -= (100.0f - machine.pump_pressure_bar) * 0.9f;
  else if (machine.pump_pressure_bar > 130.0)
    score -= (machine.pump_pressure_bar - 130.0f) * 0.7f;

  if (machine.motor_load_percent > 82.0)
    score -= (machine.motor_load_percent - 82.0f) * 0.6f;
  if (machine.motor_load_percent < 25.0)
    score -= (25.0f - machine.motor_load_percent) * 1.2f;

  float vMag = sqrtf(
    machine.vibration_x * machine.vibration_x +
    machine.vibration_y * machine.vibration_y +
    machine.vibration_z * machine.vibration_z
  );
  if (vMag > 0.50f)
    score -= (vMag - 0.50f) * 22.0f;

  if (machine.water_flow_lpm < 20.0)
    score -= (20.0f - machine.water_flow_lpm) * 1.5f;

  return clampF(score, 10.0, 100.0);
}

// ================================================================
//  MACHINE INIT
// ================================================================
void initMachineState() {
  machine.temperature_c       = rFloat(44.0, 48.0);
  machine.motor_load_percent  = rFloat(60.0, 68.0);
  machine.pump_pressure_bar   = rFloat(110.0, 118.0);
  machine.nozzle_pressure_bar = machine.pump_pressure_bar * 0.945f;
  machine.water_flow_lpm      = rFloat(23.0, 27.0);
  machine.vibration_x         = rFloat(0.18, 0.26);
  machine.vibration_y         = rFloat(0.18, 0.26);
  machine.vibration_z         = rFloat(0.20, 0.28);
  machine.runtime_hours       = rFloat(1200.0, 1280.0);
  machine.health_score        = 94.0;

  machine.target_temp         = machine.temperature_c;
  machine.target_load         = machine.motor_load_percent;
  machine.target_pressure     = machine.pump_pressure_bar;
  machine.target_flow         = machine.water_flow_lpm;
  machine.target_vib          = machine.vibration_x;

  machine.fault_type  = FAULT_NONE;
  machine.fault_code  = "normal";    // String assignment
  machine.status      = "running";   // String assignment
  machine.phase       = 0;

  Serial.println("[MACHINE] Initialized in healthy state");
}

// ================================================================
//  PHYSICS SIMULATION
// ================================================================
void updateSimulation() {
  machine.runtime_hours += (PUBLISH_MS / 1000.0f) / 3600.0f;

  // Pressure instability resamples every tick to create wild swing
  if (machine.fault_type == FAULT_PRESSURE_INSTABILITY) {
    machine.target_pressure = rFloat(55.0, 155.0);
    machine.target_flow     = rFloat(18.0, 32.0);
  }

  float lf       = (machine.fault_type == FAULT_NONE) ? 0.06f : 0.14f;
  float vibNoise = (machine.fault_type == FAULT_PRESSURE_INSTABILITY ||
                    machine.fault_type == FAULT_PUMP_DEGRADATION) ? 0.05f : 0.012f;
  float pNoise   = (machine.fault_type == FAULT_PRESSURE_INSTABILITY) ? 6.0f : 1.2f;

  machine.temperature_c      = lerpToward(machine.temperature_c,      machine.target_temp,     lf * 0.5f, 0.25f);
  machine.motor_load_percent = lerpToward(machine.motor_load_percent, machine.target_load,     lf,        0.8f);
  machine.pump_pressure_bar  = lerpToward(machine.pump_pressure_bar,  machine.target_pressure, lf * 1.2f, pNoise);
  machine.water_flow_lpm     = lerpToward(machine.water_flow_lpm,     machine.target_flow,     lf,        0.4f);
  machine.vibration_x        = lerpToward(machine.vibration_x,        machine.target_vib,      lf,        vibNoise);
  machine.vibration_y        = lerpToward(machine.vibration_y,        machine.target_vib * rFloat(0.90f, 1.10f), lf, vibNoise);
  machine.vibration_z        = lerpToward(machine.vibration_z,        machine.target_vib * rFloat(0.88f, 1.12f), lf, vibNoise);

  machine.temperature_c      = clampF(machine.temperature_c,       25.0, 100.0);
  machine.motor_load_percent = clampF(machine.motor_load_percent,  15.0, 100.0);
  machine.pump_pressure_bar  = clampF(machine.pump_pressure_bar,   30.0, 165.0);
  machine.water_flow_lpm     = clampF(machine.water_flow_lpm,       8.0,  40.0);
  machine.vibration_x        = clampF(machine.vibration_x,          0.02,  2.5);
  machine.vibration_y        = clampF(machine.vibration_y,          0.02,  2.5);
  machine.vibration_z        = clampF(machine.vibration_z,          0.02,  2.5);

  // Nozzle = 93–96% of pump pressure normally; wider gap during instability
  float ratio = (machine.fault_type == FAULT_PRESSURE_INSTABILITY)
                ? rFloat(0.78f, 0.92f) : rFloat(0.930f, 0.955f);
  machine.nozzle_pressure_bar = clampF(
    machine.pump_pressure_bar * ratio + rFloat(-1.2f, 1.2f), 30.0, 160.0
  );

  machine.health_score = computeHealthScore();
}

// ================================================================
//  FAULT SWITCH HANDLER
//  Reads all 3 switches. Acts only on STATE CHANGE.
//  Priority: SW1 > SW2 > SW3
// ================================================================
void handleFaultSwitches() {
  int sw1 = digitalRead(SW_THREAD_BREAK);
  int sw2 = digitalRead(SW_PRESSURE_INSTAB);
  int sw3 = digitalRead(SW_PUMP_DEGRAD);

  if (sw1 == lastSw1 && sw2 == lastSw2 && sw3 == lastSw3) return;

  lastSw1 = sw1;
  lastSw2 = sw2;
  lastSw3 = sw3;

  if (sw1 == LOW || sw2 == LOW || sw3 == LOW) {
    manualFaultActive = true;

    if (sw1 == LOW) {
      setThreadBreakTargets();
      Serial.println("\n[SW1] FAULT INJECTED: thread_break");
      Serial.println("      motor_load drops | flow drops | pressure spikes");

    } else if (sw2 == LOW) {
      setPressureInstabilityTargets();
      Serial.println("\n[SW2] FAULT INJECTED: pressure_instability");
      Serial.println("      pressure swings 55-155 bar | vibration rises");

    } else if (sw3 == LOW) {
      setPumpDegradationTargets();
      Serial.println("\n[SW3] FAULT INJECTED: pump_degradation");
      Serial.println("      pressure 62-78 bar | vibration 1.0-1.8g");
    }

  } else {
    manualFaultActive = false;
    setNormalTargets();
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("\n[SWITCH] All OFF — returning to normal");
  }
}

// ================================================================
//  BUZZER
// ================================================================
void handleBuzzer() {
  // machine.status is a String — String == "fault" works fine
  bool isFault = (machine.fault_type != FAULT_NONE &&
                  machine.status == "fault");
  digitalWrite(BUZZER_PIN, isFault ? HIGH : LOW);
}

// ================================================================
//  PUBLISH
// ================================================================
void publishLog() {
  StaticJsonDocument<640> doc;

  doc["timestamp"]            = makeTimestamp();
  doc["machine_id"]           = MACHINE_ID;
  doc["machine_type"]         = "water_jet_loom";
  doc["status"]               = machine.status;
  doc["phase"]                = machine.phase;
  doc["health_score"]         = serialized(String(machine.health_score,        1));
  doc["temperature_c"]        = serialized(String(machine.temperature_c,       2));
  doc["motor_load_percent"]   = serialized(String(machine.motor_load_percent,  1));
  doc["pump_pressure_bar"]    = serialized(String(machine.pump_pressure_bar,   2));
  doc["nozzle_pressure_bar"]  = serialized(String(machine.nozzle_pressure_bar, 2));
  doc["water_flow_lpm"]       = serialized(String(machine.water_flow_lpm,      2));
  doc["vibration_x"]          = serialized(String(machine.vibration_x,         3));
  doc["vibration_y"]          = serialized(String(machine.vibration_y,         3));
  doc["vibration_z"]          = serialized(String(machine.vibration_z,         3));
  doc["runtime_hours"]        = serialized(String(machine.runtime_hours,       4));
  doc["fault_code"]           = machine.fault_code;

  char payload[640];
  size_t pLen = serializeJson(doc, payload);

  bool ok = mqttClient.publish(MQTT_DATA_TOPIC, payload, pLen);

  Serial.println("\n─────────────────────────────────────────────");
  Serial.println(ok ? "[MQTT] Published OK" : "[MQTT] PUBLISH FAILED");
  Serial.println(String(payload));
  Serial.printf("  health : %.1f | fault: %s | status: %s\n",
    machine.health_score,
    machine.fault_code.c_str(),
    machine.status.c_str());
  Serial.printf("  temp=%.2fC  press=%.2fbar  flow=%.2fLPM  load=%.1f%%\n",
    machine.temperature_c, machine.pump_pressure_bar,
    machine.water_flow_lpm, machine.motor_load_percent);
  float vMag = sqrtf(machine.vibration_x*machine.vibration_x +
                     machine.vibration_y*machine.vibration_y +
                     machine.vibration_z*machine.vibration_z);
  Serial.printf("  vib mag=%.3fg  (X:%.3f Y:%.3f Z:%.3f)\n",
    vMag, machine.vibration_x, machine.vibration_y, machine.vibration_z);
  Serial.println("─────────────────────────────────────────────");
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  randomSeed(analogRead(0));
  startMillis = millis();

  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║  WATER JET LOOM EMULATOR v3.0 — CityPlus AI  ║");
  Serial.println("╠═══════════════════════════════════════════════╣");
  Serial.println("║  SW1 GPIO18 → thread_break                    ║");
  Serial.println("║  SW2 GPIO19 → pressure_instability            ║");
  Serial.println("║  SW3 GPIO21 → pump_degradation                ║");
  Serial.println("║  MQTT STOP / START / RESET_FAULT              ║");
  Serial.println("╚═══════════════════════════════════════════════╝");

  pinMode(SW_THREAD_BREAK,    INPUT_PULLUP);
  pinMode(SW_PRESSURE_INSTAB, INPUT_PULLUP);
  pinMode(SW_PUMP_DEGRAD,     INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  connectWiFi();
  connectMQTT();
  initMachineState();

  lastPublishTime = millis();
  Serial.println("\n[SYSTEM] Ready — publishing every 2s\n");
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Reconnecting...");
    connectWiFi();
  }
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] Reconnecting...");
    connectMQTT();
  }
  mqttClient.loop();

  handleFaultSwitches();
  handleBuzzer();

  unsigned long now = millis();
  if (now - lastPublishTime >= PUBLISH_MS) {
    lastPublishTime = now;
    if (publishingEnabled) {
      updateSimulation();
      publishLog();
    } else {
      Serial.println("[SYSTEM] Stopped — send START to resume");
    }
  }
}