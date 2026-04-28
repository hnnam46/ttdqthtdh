#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Daikin.h>
#include <ir_Panasonic.h>
#include <ir_LG.h>
#include <ir_Mitsubishi.h>

#include <DHT.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <EthernetUdp.h>
#include <NTPClient.h>
#include "hardware/watchdog.h"

// ================== CẤU HÌNH CHUNG ==================
#define ROLE_PICO 2                 // 1 = outdoor, 2 = indoor
#define BUILDING_ID "elb"
#define ROOM_ID     "prl"
#define NUM_AC      6
#define NUM_SENSOR  2

// ================== MQTT ==================
const char* mqtt_server = "103.82.194.179";
const uint16_t mqtt_port = 1883;
const char* mqtt_user = "hnnam46";
const char* mqtt_pass = "Namnam123";

// ================== ETHERNET ==================
#define PIN_ETH_CS 17
byte mac[] = {0xDE,0xAD,0xBE,0xEF,0xFE,0xED};
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

// ================== DHT ==================
#define DHTTYPE DHT22
#define DHT_PIN_1 4
#define DHT_PIN_2 5
DHT dht1(DHT_PIN_1, DHTTYPE);
DHT dht2(DHT_PIN_2, DHTTYPE);

// ================== OLED ==================
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire1, -1);

bool oledOn = true;
unsigned long lastOledActivity = 0;
const unsigned long OLED_TIMEOUT_MS = 120000;

float lastDisplayedTemp = NAN;
bool lastNetStatus = false;

// ================== IR ==================
#define IR_PIN 3
IRsend irsend(IR_PIN);

// ================== NTP (Google) ==================
EthernetUDP ntpUDP;
const char* ntpServer = "time.google.com";
NTPClient timeClient(ntpUDP, ntpServer, 25200, 60000); // UTC+7

bool ntpSynced = false;
unsigned long lastNtpTry = 0;

// ================== STRUCT ==================
struct SensorState {
  String name;
  float temp;
  float hum;
  bool ok;
};

struct AcState {
  String name;
  String opr_mode;      // auto / man
  String power;         // on / off
  String mode;          // cool / dry
  String speed;
  String swing;
  String fac;           // daikin / pana / lg / mitsu / casper
  float  ctrl_temp;     // nhiệt độ do HA gửi
  float  sent_temp;     // nhiệt độ sẽ phát IR
  float  prev_sent_temp;

  unsigned long lastUserCmdMs;
  unsigned long lastAutoIrMs;
  float lastAutoRoomTemp;
};

struct RoomConfig {
  String building;
  String room;
  float temp_setauto;
  float hum_setauto;
  float thr_temp;
};

RoomConfig roomCfg = {
  BUILDING_ID,
  ROOM_ID,
  27.0f,
  60.0f,
  24.0f
};

SensorState sensors[NUM_SENSOR];
AcState acs[NUM_AC];

// ================== IR QUEUE ==================
struct IrTask {
  int acIndex;
  bool fromUser;
  unsigned long nextTime;
  uint8_t retry;
};

const uint8_t IR_QUEUE_SIZE = 16;
IrTask irQueue[IR_QUEUE_SIZE];
uint8_t irHead = 0, irTail = 0;
const uint32_t IR_STAGGER_MS = 5000;   // 5s giữa các lần phát IR

// ================== NETWORK STATE ==================
bool ethernetReady = false;
bool mqttConnected = false;
uint32_t lastMqttReconnectAttempt = 0;
uint8_t mqttReconnectCount = 0;

// ================== HEARTBEAT & OUTDOOR ==================
uint32_t lastHeartbeat = 0;
float outdoorTemp = 25.0f;
unsigned long lastOutdoorReq = 0;

// ================== LOG ==================
void logMsg(const String &tag, const String &msg) {
  Serial.print("[");
  Serial.print(timeClient.getFormattedTime());
  Serial.print("] ");
  Serial.print(tag);
  Serial.print(": ");
  Serial.println(msg);
}

// ================== WATCHDOG ==================
void setupWatchdog() {
  watchdog_enable(8000, 1);
}

void feedWatchdog() {
  watchdog_update();
}

// ================== OLED ==================
SensorState* getActiveSensor() {
  if (sensors[0].ok) return &sensors[0];
  if (NUM_SENSOR > 1 && sensors[1].ok) return &sensors[1];
  return nullptr;
}

void oledWake() {
  if (!oledOn) {
    oledOn = true;
    display.ssd1306_command(SSD1306_DISPLAYON);
    logMsg("OLED", "WAKE");
  }
  lastOledActivity = millis();
}

void oledMaybeSleep() {
  if (oledOn && (millis() - lastOledActivity > OLED_TIMEOUT_MS)) {
    oledOn = false;
    display.clearDisplay();
    display.display();
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    logMsg("OLED", "SLEEP TIMEOUT");
  }
}

void setupOled() {
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();
  Wire1.setClock(400000);
  delay(200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    logMsg("OLED", "INIT FAIL");
    return;
  }

  oledOn = true;
  lastOledActivity = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED READY");
  display.display();
}

void updateOled(bool netStatus) {
  if (!oledOn) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  SensorState* active = getActiveSensor();
  float t = active ? active->temp : NAN;
  float h = active ? active->hum  : NAN;

  display.setCursor(0, 0);
  display.print("Bld: ");
  display.print(BUILDING_ID);
  display.print(" Rm: ");
  display.print(ROOM_ID);

  display.setCursor(0, 12);
  display.print("T/H: ");
  if (!isnan(t) && !isnan(h)) {
    display.print(t, 1);
    display.print("C / ");
    display.print(h, 1);
    display.print("%");
  } else {
    display.print("ERR");
  }

  display.setCursor(0, 24);
  display.print("Net: ");
  display.print(netStatus ? "ONL" : "OFF");

  display.setCursor(0, 36);
  display.print("Time: ");
  if (!ntpSynced)
    display.print("WAIT NTP");
  else
    display.print(timeClient.getFormattedTime());

  display.display();
}

// ================== ETHERNET ==================
void setupEthernet() {
  logMsg("ETH", "INIT...");
  Ethernet.init(PIN_ETH_CS);
  Ethernet.begin(mac);
  delay(1000);
  IPAddress ip = Ethernet.localIP();
  if (ip[0] == 0) {
    ethernetReady = false;
    logMsg("ETH", "NO IP (0.0.0.0)");
  } else {
    ethernetReady = true;
    logMsg("ETH", "IP = " + ip.toString());
  }
}

// ================== MQTT ==================
String makeClientId() {
  String cid = "pico_";
  cid += BUILDING_ID;
  cid += "_";
  cid += ROOM_ID;
  cid += "_";
  cid += String((uint32_t)millis(), HEX);
  return cid;
}

String availabilityTopic() {
  return String(BUILDING_ID) + "/" + ROOM_ID + "/pico/availability";
}

void publishAvailabilityOnline() {
  String key = availabilityTopic();
  mqttClient.publish(key.c_str(), "online", true);
  logMsg("LWT", "ONLINE → " + key);
}

void mqttSubscribeAll() {
  // AC control: elb/prl/ac/+/#
  String t_ac = String(BUILDING_ID) + "/" + ROOM_ID + "/ac/+/+";
  mqttClient.subscribe(t_ac.c_str());
  logMsg("MQTT SUB", t_ac);

  // Config: elb/prl/config/...
  String baseCfg = String(BUILDING_ID) + "/" + ROOM_ID + "/config/";
  mqttClient.subscribe((baseCfg + "temp_setauto").c_str());
  mqttClient.subscribe((baseCfg + "hum_setauto").c_str());
  mqttClient.subscribe((baseCfg + "thr_temp").c_str());
  logMsg("MQTT SUB", baseCfg + "temp_setauto");
  logMsg("MQTT SUB", baseCfg + "hum_setauto");
  logMsg("MQTT SUB", baseCfg + "thr_temp");

  // Outdoor temp: elb/outdoor/sensor/1/temp
  String t_env = String(BUILDING_ID) + "/outdoor/sensor/1/temp";
  mqttClient.subscribe(t_env.c_str());
  logMsg("MQTT SUB", t_env);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  logMsg("MQTT RX", t + " = " + msg);

  // Tách topic theo '/'
  // Ví dụ: elb/prl/ac/1/power
  //        0   1   2  3  4
  int p1 = t.indexOf('/');
  int p2 = t.indexOf('/', p1 + 1);
  int p3 = t.indexOf('/', p2 + 1);
  int p4 = t.indexOf('/', p3 + 1);

  String lvl0 = (p1 > 0) ? t.substring(0, p1) : t;
  String lvl1 = (p1 > 0 && p2 > p1) ? t.substring(p1 + 1, p2) : "";
  String lvl2 = (p2 > 0 && p3 > p2) ? t.substring(p2 + 1, p3) : "";
  String lvl3 = (p3 > 0 && p4 > p3) ? t.substring(p3 + 1, p4) : "";
  String lvl4 = (p4 > 0) ? t.substring(p4 + 1) : "";

  // AC control: elb/prl/ac/<id>/<param>
  if (lvl0 == BUILDING_ID && lvl1 == ROOM_ID && lvl2 == "ac") {
    int acIndex = lvl3.toInt() - 1;
    if (acIndex >= 0 && acIndex < NUM_AC) {
      AcState &ac = acs[acIndex];

      String param = lvl4;
      String oldMode = ac.mode;
      String oldPower = ac.power;
      float oldCtrl = ac.ctrl_temp;

      if (param == "opr") {
        ac.opr_mode = msg;
      } else if (param == "power") {
        ac.power = msg;
      } else if (param == "mode") {
        ac.mode = msg;
      } else if (param == "speed") {
        ac.speed = msg;
      } else if (param == "swing") {
        ac.swing = msg;
      } else if (param == "fac") {
        ac.fac = msg;
      } else if (param == "ctrl_temp") {
        ac.ctrl_temp = msg.toFloat();
      }

      ac.lastUserCmdMs = millis();

      if (ac.mode != oldMode || ac.power != oldPower || fabs(ac.ctrl_temp - oldCtrl) >= 0.1f) {
        logMsg("AUTO CHANGE",
               ac.name + " USER → opr=" + ac.opr_mode +
               " power=" + ac.power +
               " mode=" + ac.mode +
               " fac=" + ac.fac +
               " ctrl_temp=" + String(ac.ctrl_temp));
      }

      uint8_t nextTail = (irTail + 1) % IR_QUEUE_SIZE;
      if (nextTail != irHead) {
        irQueue[irTail].acIndex = acIndex;
        irQueue[irTail].fromUser = true;
        irQueue[irTail].retry = 0;
        irQueue[irTail].nextTime = millis();
        irTail = nextTail;
        logMsg("IR QUEUE", "ENQUEUE USER → " + ac.name);
      } else {
        logMsg("IR QUEUE", "FULL (USER CMD) → BO QUA");
      }

      oledWake();
    }
    return;
  }

  // Config: elb/prl/config/<param>
  if (lvl0 == BUILDING_ID && lvl1 == ROOM_ID && lvl2 == "config") {
    String param = lvl3;
    if (param == "temp_setauto") {
      float old = roomCfg.temp_setauto;
      roomCfg.temp_setauto = msg.toFloat();
      if (fabs(roomCfg.temp_setauto - old) >= 0.1f)
        logMsg("AUTO CHANGE", "temp_setauto = " + String(roomCfg.temp_setauto));
      oledWake();
    } else if (param == "hum_setauto") {
      float old = roomCfg.hum_setauto;
      roomCfg.hum_setauto = msg.toFloat();
      if (fabs(roomCfg.hum_setauto - old) >= 0.1f)
        logMsg("AUTO CHANGE", "hum_setauto = " + String(roomCfg.hum_setauto));
      oledWake();
    } else if (param == "thr_temp") {
      float old = roomCfg.thr_temp;
      roomCfg.thr_temp = msg.toFloat();
      if (fabs(roomCfg.thr_temp - old) >= 0.1f)
        logMsg("AUTO CHANGE", "thr_temp = " + String(roomCfg.thr_temp));
      oledWake();
    }
    return;
  }

  // Outdoor temp: elb/outdoor/sensor/1/temp
  if (lvl0 == BUILDING_ID && lvl1 == "outdoor" && lvl2 == "sensor" && lvl3 == "1" && lvl4 == "temp") {
    float old = outdoorTemp;
    outdoorTemp = msg.toFloat();
    if (fabs(outdoorTemp - old) >= 0.5f)
      logMsg("SENSOR CHANGE", "OUTDOOR TEMP = " + String(outdoorTemp));
    return;
  }
}

void mqttReconnect() {
  if (!ethernetReady) return;
  if (mqttClient.connected()) return;

  uint32_t now = millis();
  if (now - lastMqttReconnectAttempt < 8000) return;
  lastMqttReconnectAttempt = now;

  logMsg("MQTT", "RECONNECTING...");

  String clientId = makeClientId();
  String lwtTopic = availabilityTopic();

  if (mqttClient.connect(clientId.c_str(),
                         mqtt_user, mqtt_pass,
                         lwtTopic.c_str(), 0, true, "offline")) {
    mqttConnected = true;
    mqttReconnectCount = 0;
    logMsg("MQTT", "CONNECTED");
    publishAvailabilityOnline();
    if (ROLE_PICO == 2) mqttSubscribeAll();
    oledWake();
  } else {
    mqttConnected = false;
    mqttReconnectCount++;
    logMsg("MQTT", "FAIL (" + String(mqttReconnectCount) + ")");
  }
}

// ================== NTP HANDLE ==================
void handleNtp() {
  static bool mqttWasOnline = false;
  static unsigned long mqttOnlineAt = 0;

  if (!ethernetReady) return;
  if (!mqttClient.connected()) {
    mqttWasOnline = false;
    return;
  }

  if (!mqttWasOnline) {
    mqttWasOnline = true;
    mqttOnlineAt = millis();
    return;
  }

  if (millis() - mqttOnlineAt < 3000) return;

  if (millis() - lastNtpTry > 10000) {
    lastNtpTry = millis();
    if (timeClient.forceUpdate()) {
      if (!ntpSynced) {
        ntpSynced = true;
        logMsg("NTP SYNC", timeClient.getFormattedTime());
      } else {
        logMsg("NTP UPDATE", timeClient.getFormattedTime());
      }
    } else {
      logMsg("NTP FAIL", "DNS/UDP CHUA SAN SANG");
    }
  }
}

// ================== SENSOR ==================
void publishSensorError() {
  String topic = String(BUILDING_ID) + "/" + ROOM_ID + "/sensor/error";
  String payload = "sensor_fail";
  if (mqttClient.connected()) {
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
  }
  logMsg("ERROR SENSOR", "PUBLISH → " + topic + " = " + payload);
}

void readSensorsOutdoor() {
  float t1 = dht1.readTemperature();
  float h1 = dht1.readHumidity();
  float t2 = dht2.readTemperature();
  float h2 = dht2.readHumidity();

  sensors[0].name = "ssenv1";
  sensors[0].temp = t1;
  sensors[0].hum  = h1;
  sensors[0].ok   = !isnan(t1) && !isnan(h1);

  sensors[1].name = "ssenv2";
  sensors[1].temp = t2;
  sensors[1].hum  = h2;
  sensors[1].ok   = !isnan(t2) && !isnan(h2);

  static float lastT1 = NAN, lastH1 = NAN, lastT2 = NAN, lastH2 = NAN;
  if (sensors[0].ok && (isnan(lastT1) || fabs(t1 - lastT1) >= 0.5f || fabs(h1 - lastH1) >= 2.0f)) {
    logMsg("SENSOR CHANGE", "ssenv1 T=" + String(t1) + " H=" + String(h1));
    lastT1 = t1; lastH1 = h1;
  }
  if (sensors[1].ok && (isnan(lastT2) || fabs(t2 - lastT2) >= 0.5f || fabs(h2 - lastH2) >= 2.0f)) {
    logMsg("SENSOR CHANGE", "ssenv2 T=" + String(t2) + " H=" + String(h2));
    lastT2 = t2; lastH2 = h2;
  }
}

void readSensorsIndoor() {
  float t1 = dht1.readTemperature();
  float h1 = dht1.readHumidity();
  sensors[0].name = "ssrom1";
  sensors[0].temp = t1;
  sensors[0].hum  = h1;
  sensors[0].ok   = !isnan(t1) && !isnan(h1);

  if (NUM_SENSOR > 1) {
    float t2 = dht2.readTemperature();
    float h2 = dht2.readHumidity();
    sensors[1].name = "ssrom2";
    sensors[1].temp = t2;
    sensors[1].hum  = h2;
    sensors[1].ok   = !isnan(t2) && !isnan(h2);
  }

  SensorState* active = getActiveSensor();
  static float lastT = NAN, lastH = NAN;
  if (active && active->ok) {
    if (isnan(lastT) || fabs(active->temp - lastT) >= 0.5f ||
        isnan(lastH) || fabs(active->hum - lastH) >= 2.0f) {
      logMsg("SENSOR CHANGE",
             active->name + " T=" + String(active->temp) +
             " H=" + String(active->hum));
      lastT = active->temp;
      lastH = active->hum;
    }
  } else {
    logMsg("ERROR SENSOR", "KHONG CO SENSOR HOP LE");
  }
}

// ================== AUTO LOGIC ==================
void computeAutoForAc(AcState &ac, float t_outdoor, float t_room, float h_room, int index) {
  if (ac.opr_mode == "man") {
    ac.sent_temp = ac.ctrl_temp;
    return;
  }

  float oldSent = ac.sent_temp;
  String oldMode = ac.mode;

  float targetTemp = roomCfg.temp_setauto;
  String mode = "cool";

  if (t_outdoor > 27.0f) {
    targetTemp = roomCfg.temp_setauto;
  } else if (t_outdoor < roomCfg.thr_temp) {
    mode = "dry";
    targetTemp = t_outdoor + 2.0f;
  }

  if (h_room > roomCfg.hum_setauto) {
    mode = "dry";
  } else if (h_room < 20.0f) {
    mode = "cool";
  }

  ac.mode = mode;
  ac.sent_temp = targetTemp;

  if (fabs(ac.sent_temp - oldSent) >= 0.5f || ac.mode != oldMode) {
    logMsg("AUTO CHANGE",
           ac.name + " AUTO → mode=" + ac.mode +
           " sent_temp=" + String(ac.sent_temp) +
           " (t_out=" + String(t_outdoor) +
           " t_room=" + String(t_room) +
           " h_room=" + String(h_room) + ")");
  }

  if (fabs(ac.sent_temp - oldSent) >= 0.5f) {
    uint8_t nextTail = (irTail + 1) % IR_QUEUE_SIZE;
    if (nextTail != irHead) {
      irQueue[irTail].acIndex = index;
      irQueue[irTail].fromUser = false;
      irQueue[irTail].retry = 0;
      irQueue[irTail].nextTime = millis();
      irTail = nextTail;
      logMsg("IR QUEUE", "ENQUEUE AUTO → " + ac.name);
      ac.lastAutoIrMs = millis();
      ac.lastAutoRoomTemp = t_room;
    } else {
      logMsg("IR QUEUE", "FULL (AUTO) → BO QUA");
    }
  }
}

// ================== IR SEND ==================
void sendIrForAc(AcState &ac) {
  if (ac.power != "on") {
    logMsg("IR SEND", ac.name + " POWER OFF → KHONG PHAT IR");
    return;
  }

  logMsg("IR SEND",
         ac.name + " fac=" + ac.fac +
         " mode=" + ac.mode +
         " temp=" + String(ac.sent_temp) +
         " swing=" + ac.swing);

  uint8_t t = (uint8_t)ac.sent_temp;
  bool isCool = (ac.mode == "cool");

  if (ac.fac == "daikin") {
    IRDaikinESP ir(IR_PIN);
    ir.begin();
    ir.setPower(true);
    ir.setTemp(t);
    ir.setMode(isCool ? kDaikinCool : kDaikinDry);
    ir.setFan(kDaikinFanAuto);
    ir.setSwingVertical(ac.swing == "sw-auto" ? kDaikinSwingOn : kDaikinSwingOff);
    ir.send();
    return;
  }

  if (ac.fac == "pana") {
    IRPanasonicAc ir(IR_PIN);
    ir.begin();
    ir.setPower(true);
    ir.setTemp(t);
    ir.setMode(isCool ? kPanasonicAcCool : kPanasonicAcDry);
    ir.setFan(kPanasonicAcFanAuto);
    ir.setSwingVertical(ac.swing == "sw-auto");
    ir.send();
    return;
  }

  if (ac.fac == "lg") {
    IRLgAc ir(IR_PIN);
    ir.begin();
    ir.setPower(true);
    ir.setTemp(t);
    ir.setMode(isCool ? kLgAcCool : kLgAcDry);
    ir.setFan(kLgAcFanAuto);
    ir.send();
    return;
  }

  if (ac.fac == "mitsu") {
    IRMitsubishiAC ir(IR_PIN);
    ir.begin();
    ir.setPower(true);
    ir.setTemp(t);
    ir.setMode(isCool ? kMitsubishiAcCool : kMitsubishiAcDry);
    ir.setFan(kMitsubishiAcFanAuto);
    ir.setVane(kMitsubishiAcVaneAuto);
    ir.send();
    return;
  }

  if (ac.fac == "casper") {
    logMsg("IR SEND", "Casper: CHUA CO MA RAW, CAN BO SUNG SAU");
    return;
  }

  logMsg("IR SEND", "HANG KHONG HO TRO: " + ac.fac);
}

// ================== IR QUEUE PROCESS ==================
void processIrQueue() {
  if (irHead == irTail) return;

  IrTask &task = irQueue[irHead];
  if (millis() < task.nextTime) return;

  logMsg("IR QUEUE",
         "PROCESS acIndex=" + String(task.acIndex) +
         " retry=" + String(task.retry) +
         " fromUser=" + String(task.fromUser ? "Y" : "N"));

  sendIrForAc(acs[task.acIndex]);

  task.retry++;
  if (task.retry < 2) {
    task.nextTime = millis() + IR_STAGGER_MS;
    logMsg("IR QUEUE", "SCHEDULE NEXT RETRY SAU 5s");
  } else {
    irHead = (irHead + 1) % IR_QUEUE_SIZE;
    logMsg("IR QUEUE", "DONE TASK → POP");
  }
}

// ================== JSON PUB ==================
void publishRoomState() {
  StaticJsonDocument<4096> doc;

  JsonArray jsSensors = doc.createNestedArray("sensors");
  for (int i = 0; i < NUM_SENSOR; i++) {
    JsonObject o = jsSensors.createNestedObject();
    o["name"] = sensors[i].name;
    o["temp"] = sensors[i].temp;
    o["hum"]  = sensors[i].hum;
    o["ok"]   = sensors[i].ok;
  }

  JsonArray jsAcs = doc.createNestedArray("acs");
  for (int i = 0; i < NUM_AC; i++) {
    JsonObject o = jsAcs.createNestedObject();
    o["name"]      = acs[i].name;
    o["opr"]       = acs[i].opr_mode;
    o["power"]     = acs[i].power;
    o["mode"]      = acs[i].mode;
    o["speed"]     = acs[i].speed;
    o["swing"]     = acs[i].swing;
    o["fac"]       = acs[i].fac;
    o["ctrl_temp"] = acs[i].ctrl_temp;
    o["sent_temp"] = acs[i].sent_temp;
  }

  JsonObject cfg = doc.createNestedObject("cfg");
  cfg["temp_setauto"] = roomCfg.temp_setauto;
  cfg["hum_setauto"]  = roomCfg.hum_setauto;
  cfg["thr_temp"]     = roomCfg.thr_temp;

  char buffer[4096];
  size_t len = serializeJson(doc, buffer, sizeof(buffer));

  String topic = String(BUILDING_ID) + "/" + ROOM_ID + "/state";

  if (!mqttClient.connected()) {
    logMsg("MQTT", "OFFLINE → KHONG GUI JSON");
    return;
  }

  bool ok = mqttClient.publish(topic.c_str(), buffer);
  logMsg("MQTT", String("PUBLISH STATE → ") + (ok ? "OK" : "FAIL"));
}

// ================== SETUP SENSORS & AC ==================
void setupSensors() {
  dht1.begin();
  if (NUM_SENSOR > 1) dht2.begin();
}

void setupAcs() {
  for (int i = 0; i < NUM_AC; i++) {
    acs[i].name      = "ac" + String(i + 1);
    acs[i].opr_mode  = "auto";
    acs[i].power     = "off";
    acs[i].mode      = "cool";
    acs[i].speed     = "auto";
    acs[i].swing     = "sw-auto";
    acs[i].fac       = "daikin";
    acs[i].ctrl_temp = 27.0f;
    acs[i].sent_temp = 27.0f;
    acs[i].prev_sent_temp = 27.0f;
    acs[i].lastUserCmdMs = 0;
    acs[i].lastAutoIrMs = 0;
    acs[i].lastAutoRoomTemp = NAN;
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("===== PICO BMS (NET + MQTT + NTP + IR SAFE, TREE TOPIC) =====");

  setupWatchdog();
  setupEthernet();

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  setupSensors();
  setupAcs();
  setupOled();

  irsend.begin();
  timeClient.begin();
}

// ================== LOOP ==================
void loop() {
  feedWatchdog();

  // Heartbeat đơn giản
  static unsigned long lastBeat = 0;
  if (millis() - lastBeat > 2000) {
    lastBeat = millis();
    Serial.print("[HB] loop alive, ms=");
    Serial.println(millis());
  }

  // Ethernet link
  if (Ethernet.linkStatus() == LinkOFF) {
    if (ethernetReady) {
      ethernetReady = false;
      logMsg("ETH", "LINK OFF");
    }
  } else {
    if (!ethernetReady) {
      logMsg("ETH", "LINK ON → REINIT");
      setupEthernet();
      oledWake();
    }
  }

  // MQTT
  bool prevConn = mqttConnected;
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();
  mqttConnected = mqttClient.connected();
  if (!mqttConnected && prevConn) {
    logMsg("MQTT", "DISCONNECTED SAU LOOP");
  }

  // NTP
  handleNtp();

  // NET CHANGE
  bool netStatus = mqttClient.connected();
  static bool lastNet = false;
  if (netStatus != lastNet) {
    lastNet = netStatus;
    logMsg("NET CHANGE", netStatus ? "ONLINE" : "OFFLINE");
    oledWake();
  }

  // SUB/REQ outdoor mỗi 1 phút
  if (ROLE_PICO == 2 && mqttClient.connected()) {
    if (millis() - lastOutdoorReq > 60000) {
      String reqTopic = String(BUILDING_ID) + "/outdoor/request";
      mqttClient.publish(reqTopic.c_str(), "1");
      logMsg("MQTT", "REQUEST OUTDOOR → " + reqTopic);
      lastOutdoorReq = millis();
    }
  }

  // Read sensors
  if (ROLE_PICO == 1) {
    readSensorsOutdoor();
  } else {
    readSensorsIndoor();
  }

  // Auto-failover sensor cho phòng chỉ có 1 sensor
  if (ROLE_PICO == 2 && NUM_SENSOR == 1 && !sensors[0].ok) {
    logMsg("ERROR SENSOR", "ssrom1 FAIL → DUNG AC");
    publishSensorError();
    for (int i = 0; i < NUM_AC; i++) {
      acs[i].power = "off";
    }
    updateOled(netStatus);
    oledMaybeSleep();
    delay(500);
    return;
  }

  // Sự kiện nhiệt độ thay đổi ≥1°C → wake OLED
  SensorState* active = getActiveSensor();
  if (active && !isnan(active->temp)) {
    if (isnan(lastDisplayedTemp) || fabs(active->temp - lastDisplayedTemp) >= 1.0f) {
      logMsg("SENSOR CHANGE", "ROOM TEMP Δ≥1C → WAKE OLED");
      lastDisplayedTemp = active->temp;
      oledWake();
    }
  }

  // Indoor logic + IR queue
  if (ROLE_PICO == 2) {
    if (active && active->ok) {
      float t_room = active->temp;
      float h_room = active->hum;
      float t_outdoor = outdoorTemp;

      for (int i = 0; i < NUM_AC; i++) {
        acs[i].prev_sent_temp = acs[i].sent_temp;
        computeAutoForAc(acs[i], t_outdoor, t_room, h_room, i);
      }

      // AUTO-ADJUST sau 5 phút nếu phòng không mát
      unsigned long now = millis();
      for (int i = 0; i < NUM_AC; i++) {
        AcState &ac = acs[i];
        if (ac.opr_mode == "auto" && ac.power == "on") {
          if (ac.lastAutoIrMs > 0 && (now - ac.lastAutoIrMs > 300000)) { // 5 phút
            if (!isnan(ac.lastAutoRoomTemp) &&
                fabs(t_room - ac.lastAutoRoomTemp) < 0.5f) {
              ac.sent_temp -= 1.0f;
              logMsg("AUTO CHANGE",
                     ac.name + " AUTO-ADJUST → giam 1C, sent_temp=" +
                     String(ac.sent_temp));

              uint8_t nextTail = (irTail + 1) % IR_QUEUE_SIZE;
              if (nextTail != irHead) {
                irQueue[irTail].acIndex = i;
                irQueue[irTail].fromUser = false;
                irQueue[irTail].retry = 0;
                irQueue[irTail].nextTime = millis();
                irTail = nextTail;
                logMsg("IR QUEUE", "ENQUEUE AUTO-ADJUST → " + ac.name);
                ac.lastAutoIrMs = millis();
                ac.lastAutoRoomTemp = t_room;
              } else {
                logMsg("IR QUEUE", "FULL (AUTO-ADJUST) → BO QUA");
              }
            } else {
              ac.lastAutoIrMs = now;
              ac.lastAutoRoomTemp = t_room;
            }
          }
        }
      }
    }

    processIrQueue();
  }

  // Publish state mỗi 20s
  static uint32_t lastPub = 0;
  if (millis() - lastPub > 20000) {
    lastPub = millis();
    publishRoomState();
  }

  // OLED
  updateOled(netStatus);
  oledMaybeSleep();
}
