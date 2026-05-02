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
//Biến gọi debug monitor - giảm quá tải cpu do monitor quá nhiều
#define DEBUG 0

// ================== CẤU HÌNH CHUNG ==================
#define ROLE_PICO 2
#define BUILDING_ID "elb"
#define ROOM_ID     "prl"
#define NUM_AC      6
#define NUM_SENSOR  2

// ================== MQTT ==================
const char* mqtt_server = "103.82.194.179";
const uint16_t mqtt_port = 1883;
const char* mqtt_user = "hnnam46";
const char* mqtt_pass = "Namnam123@";

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

// ================== IR ==================
#define IR_PIN 3
IRsend irsend(IR_PIN);

// ================== NTP ==================
EthernetUDP ntpUDP;
const char* ntpServer = "time.google.com";
NTPClient timeClient(ntpUDP, ntpServer, 25200, 60000);

// ================== STRUCT ==================
struct SensorState {
  String name;
  float temp;
  float hum;
  bool ok;
};

struct AcState {
  String name;
  String opr_mode;
  String power;
  String mode;
  String speed;
  String swing;
  String fac;
  float ctrl_temp;
  float sent_temp;
  float prev_sent_temp;

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

// ================== IR QUEUE ==================
struct IrTask {
  int acIndex;
  bool fromUser;
  unsigned long nextTime;
  uint8_t retry;
};

class IrQueueManager {
public:
  static const uint8_t IR_QUEUE_SIZE = 16;
  static const uint32_t IR_STAGGER_MS = 5000;

  IrQueueManager() : head(0), tail(0) {}

  bool enqueue(int acIndex, bool fromUser) {
    uint8_t nextTail = (tail + 1) % IR_QUEUE_SIZE;
    if (nextTail == head) {
      log("IR QUEUE", "FULL → BO QUA");
      return false;
    }
    queue[tail].acIndex = acIndex;
    queue[tail].fromUser = fromUser;
    queue[tail].retry = 0;
    queue[tail].nextTime = millis();
    tail = nextTail;
    return true;
  }

  void process(AcState acs[], void (*sendIr)(AcState&)) {
    if (head == tail) return;
    IrTask &task = queue[head];
    if (millis() < task.nextTime) return;

    log("IR QUEUE",
        "PROCESS acIndex=" + String(task.acIndex) +
        " retry=" + String(task.retry) +
        " fromUser=" + String(task.fromUser ? "Y" : "N"));

    sendIr(acs[task.acIndex]);

    task.retry++;
    if (task.retry < 2) {
      task.nextTime = millis() + IR_STAGGER_MS;
      log("IR QUEUE", "SCHEDULE NEXT RETRY SAU 5s");
    } else {
      head = (head + 1) % IR_QUEUE_SIZE;
      log("IR QUEUE", "DONE TASK → POP");
    }
  }

private:
  IrTask queue[IR_QUEUE_SIZE];
  uint8_t head;
  uint8_t tail;

  static void log(const String &tag, const String &msg) {
    Serial.print("[");
    Serial.print(timeClient.getFormattedTime());
    Serial.print("] ");
    Serial.print(tag);
    Serial.print(": ");
    Serial.println(msg);
  }
};

// ================== OLED MANAGER ==================
class OledManager {
public:
  OledManager()
    : oledOn(true),
      lastOledActivity(0),
      lastDisplayedTemp(NAN),
      lastNetStatus(false) {}

  void setup() {
    Wire1.setSDA(6);
    Wire1.setSCL(7);
    Wire1.begin();
    Wire1.setClock(400000);
    delay(200);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      log("OLED", "INIT FAIL");
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

  void wake() {
    if (!oledOn) {
      oledOn = true;
      display.ssd1306_command(SSD1306_DISPLAYON);
      log("OLED", "WAKE");
    }
    lastOledActivity = millis();
  }

  void maybeSleep() {
    if (oledOn && (millis() - lastOledActivity > OLED_TIMEOUT_MS)) {
      oledOn = false;
      display.clearDisplay();
      display.display();
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      log("OLED", "SLEEP TIMEOUT");
    }
  }

  void update(bool netStatus, SensorState *active, bool ntpSynced) {
    if (!oledOn) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

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

  void onTempChangeWake(float currentTemp) {
    if (isnan(lastDisplayedTemp) || fabs(currentTemp - lastDisplayedTemp) >= 1.0f) {
      log("SENSOR CHANGE", "ROOM TEMP Δ≥1C → WAKE OLED");
      lastDisplayedTemp = currentTemp;
      wake();
    }
  }

private:
  bool oledOn;
  unsigned long lastOledActivity;
  float lastDisplayedTemp;
  bool lastNetStatus;

  static const unsigned long OLED_TIMEOUT_MS = 120000;

  static void log(const String &tag, const String &msg) {
    Serial.print("[");
    Serial.print(timeClient.getFormattedTime());
    Serial.print("] ");
    Serial.print(tag);
    Serial.print(": ");
    Serial.println(msg);
  }
};

// ================== SENSOR MANAGER ==================
class SensorManager {
public:
  SensorManager() {}

  void setup() {
    dht1.begin();
    if (NUM_SENSOR > 1) dht2.begin();
  }

  void readOutdoor(SensorState sensors[NUM_SENSOR]) {
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
      log("SENSOR CHANGE", "ssenv1 T=" + String(t1) + " H=" + String(h1));
      lastT1 = t1; lastH1 = h1;
    }
    if (sensors[1].ok && (isnan(lastT2) || fabs(t2 - lastT2) >= 0.5f || fabs(h2 - lastH2) >= 2.0f)) {
      log("SENSOR CHANGE", "ssenv2 T=" + String(t2) + " H=" + String(h2));
      lastT2 = t2; lastH2 = h2;
    }
  }

  void readIndoor(SensorState sensors[NUM_SENSOR]) {
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

    SensorState* active = getActiveSensor(sensors);
    static float lastT = NAN, lastH = NAN;
    if (active && active->ok) {
      if (isnan(lastT) || fabs(active->temp - lastT) >= 0.5f ||
          isnan(lastH) || fabs(active->hum - lastH) >= 2.0f) {
        log("SENSOR CHANGE",
            active->name + " T=" + String(active->temp) +
            " H=" + String(active->hum));
        lastT = active->temp;
        lastH = active->hum;
      }
    } else {
      log("ERROR SENSOR", "KHONG CO SENSOR HOP LE");
    }
  }

  SensorState* getActiveSensor(SensorState sensors[NUM_SENSOR]) {
    if (sensors[0].ok) return &sensors[0];
    if (NUM_SENSOR > 1 && sensors[1].ok) return &sensors[1];
    return nullptr;
  }

private:
  static void log(const String &tag, const String &msg) {
    Serial.print("[");
    Serial.print(timeClient.getFormattedTime());
    Serial.print("] ");
    Serial.print(tag);
    Serial.print(": ");
    Serial.println(msg);
  }
};
// ================== AC MANAGER ==================
class AcManager {
public:
  AcManager(RoomConfig &cfg, IrQueueManager &queue)
    : roomCfg(cfg), irQueue(queue), outdoorTemp(25.0f) {}

  void setup(AcState acs[NUM_AC]) {
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

  void computeAuto(AcState acs[NUM_AC], SensorState *active) {
    if (!active || !active->ok) return;

    float t_room = active->temp;
    float h_room = active->hum;
    float t_out = outdoorTemp;

    for (int i = 0; i < NUM_AC; i++) {
      acs[i].prev_sent_temp = acs[i].sent_temp;
      computeAutoForAc(acs[i], t_out, t_room, h_room, i);
    }

    unsigned long now = millis();
    for (int i = 0; i < NUM_AC; i++) {
      AcState &ac = acs[i];

      if (ac.opr_mode == "auto" && ac.power == "on") {

        // ================== IR FAIL CHECK 5 PHÚT ==================
        if (ac.lastAutoIrMs > 0 && (now - ac.lastAutoIrMs > 300000)) {
          if (!isnan(ac.lastAutoRoomTemp) &&
              fabs(t_room - ac.lastAutoRoomTemp) < 0.5f) {

            String errTopic = String(BUILDING_ID) + "/" + ROOM_ID +
                              "/ac/" + String(i + 1) + "/error";

            mqttClient.publish(errTopic.c_str(), "ir_fail", true);

            log("IR FAIL", ac.name + " KHONG THAY DOI NHIET DO SAU 5 PHÚT");
          }

          ac.lastAutoIrMs = now;
          ac.lastAutoRoomTemp = t_room;
        }
      }
    }
  }

  void setOutdoorTemp(float t) {
    float old = outdoorTemp;
    outdoorTemp = t;
    if (fabs(outdoorTemp - old) >= 0.5f) {
      log("SENSOR CHANGE", "OUTDOOR TEMP = " + String(outdoorTemp));
    }
  }

  float getOutdoorTemp() const {
    return outdoorTemp;
  }

  void applyUserCmd(AcState &ac, const String &param, const String &msg, int index) {
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
      log("AUTO CHANGE",
          ac.name + " USER → opr=" + ac.opr_mode +
          " power=" + ac.power +
          " mode=" + ac.mode +
          " fac=" + ac.fac +
          " ctrl_temp=" + String(ac.ctrl_temp));
    }

    if (irQueue.enqueue(index, true)) {
      log("IR QUEUE", "ENQUEUE USER → " + ac.name);
    }
  }

  void updateConfig(const String &param, const String &msg) {
    if (param == "temp_setauto") {
      float old = roomCfg.temp_setauto;
      roomCfg.temp_setauto = msg.toFloat();
      if (fabs(roomCfg.temp_setauto - old) >= 0.1f)
        log("AUTO CHANGE", "temp_setauto = " + String(roomCfg.temp_setauto));

    } else if (param == "hum_setauto") {
      float old = roomCfg.hum_setauto;
      roomCfg.hum_setauto = msg.toFloat();
      if (fabs(roomCfg.hum_setauto - old) >= 0.1f)
        log("AUTO CHANGE", "hum_setauto = " + String(roomCfg.hum_setauto));

    } else if (param == "thr_temp") {
      float old = roomCfg.thr_temp;
      roomCfg.thr_temp = msg.toFloat();
      if (fabs(roomCfg.thr_temp - old) >= 0.1f)
        log("AUTO CHANGE", "thr_temp = " + String(roomCfg.thr_temp));
    }
  }

  void sendIr(AcState &ac) {
    if (ac.power != "on") {
      log("IR SEND", ac.name + " POWER OFF → KHONG PHAT IR");
      return;
    }

    log("IR SEND",
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
      log("IR SEND", "Casper: CHUA CO MA RAW");
      return;
    }

    log("IR SEND", "HANG KHONG HO TRO: " + ac.fac);
  }

private:
  RoomConfig &roomCfg;
  IrQueueManager &irQueue;
  float outdoorTemp;

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

    // ================== AUTO → TỰ BẬT POWER ==================
    if (ac.opr_mode == "auto") {
      if (ac.power != "on") {
        ac.power = "on";
        log("AUTO POWER", ac.name + " → BAT POWER DO AUTO");
        irQueue.enqueue(index, false);
      }
    }

    // ================== IR FAIL CHECK 5 PHÚT ==================
    if (ac.lastAutoIrMs > 0 && (millis() - ac.lastAutoIrMs > 300000)) {
      if (!isnan(ac.lastAutoRoomTemp) &&
          fabs(t_room - ac.lastAutoRoomTemp) < 0.5f) {

        String errTopic = String(BUILDING_ID) + "/" + ROOM_ID +
                          "/ac/" + String(index + 1) + "/error";

        mqttClient.publish(errTopic.c_str(), "ir_fail", true);

        log("IR FAIL", ac.name + " KHONG THAY DOI NHIET DO SAU 5 PHÚT");
      }

      ac.lastAutoIrMs = millis();
      ac.lastAutoRoomTemp = t_room;
    }

    if (fabs(ac.sent_temp - oldSent) >= 0.5f || ac.mode != oldMode) {
      log("AUTO CHANGE",
          ac.name + " AUTO → mode=" + ac.mode +
          " sent_temp=" + String(ac.sent_temp) +
          " (t_out=" + String(t_outdoor) +
          " t_room=" + String(t_room) +
          " h_room=" + String(h_room) + ")");
    }

    if (fabs(ac.sent_temp - oldSent) >= 0.5f) {
      if (irQueue.enqueue(index, false)) {
        log("IR QUEUE", "ENQUEUE AUTO → " + ac.name);
        ac.lastAutoIrMs = millis();
        ac.lastAutoRoomTemp = t_room;
      }
    }
  }

  static void log(const String &tag, const String &msg) {
    Serial.print("[");
    Serial.print(timeClient.getFormattedTime());
    Serial.print("] ");
    Serial.print(tag);
    Serial.print(": ");
    Serial.println(msg);
  }
};

// ================== NETWORK / MQTT MANAGER ==================
class NetworkManager {
public:
  NetworkManager(RoomConfig &cfg,
                 SensorState sensors[NUM_SENSOR],
                 AcState acs[NUM_AC],
                 AcManager &acMgr,
                 OledManager &oledMgr,
                 IrQueueManager &irQueue)
    : ethernetReady(false),
      mqttConnected(false),
      lastMqttReconnectAttempt(0),
      mqttReconnectCount(0),
      lastOutdoorReq(0),
      ntpSynced(false),
      roomCfg(cfg),
      sensorsRef(sensors),
      acsRef(acs),
      acManager(acMgr),
      oled(oledMgr),
      irQueueMgr(irQueue) {}

  void setup() {
    setupEthernet();
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallbackStatic);
    timeClient.begin();
  }

  void loop() {
    handleEthernetLink();
    handleMqtt();
    handleNtp();
    handleOutdoorRequest();
  }

  bool isMqttConnected() const { return mqttConnected; }
  bool isEthernetReady() const { return ethernetReady; }
  bool isNtpSynced() const { return ntpSynced; }
//

//
  void publishRoomState() {
    StaticJsonDocument<4096> doc;

    JsonArray jsSensors = doc.createNestedArray("sensors");
    for (int i = 0; i < NUM_SENSOR; i++) {
      JsonObject o = jsSensors.createNestedObject();
      o["name"] = sensorsRef[i].name;
      o["temp"] = sensorsRef[i].temp;
      o["hum"]  = sensorsRef[i].hum;
      o["ok"]   = sensorsRef[i].ok;
    }

    JsonArray jsAcs = doc.createNestedArray("acs");
    for (int i = 0; i < NUM_AC; i++) {
      JsonObject o = jsAcs.createNestedObject();
      o["name"]      = acsRef[i].name;
      o["opr"]       = acsRef[i].opr_mode;
      o["power"]     = acsRef[i].power;
      o["mode"]      = acsRef[i].mode;
      o["speed"]     = acsRef[i].speed;
      o["swing"]     = acsRef[i].swing;
      o["fac"]       = acsRef[i].fac;
      o["ctrl_temp"] = acsRef[i].ctrl_temp;
      o["sent_temp"] = acsRef[i].sent_temp;
    }

    JsonObject cfg = doc.createNestedObject("cfg");
    cfg["temp_setauto"] = roomCfg.temp_setauto;
    cfg["hum_setauto"]  = roomCfg.hum_setauto;
    cfg["thr_temp"]     = roomCfg.thr_temp;

    char buffer[4096];
    size_t len = serializeJson(doc, buffer, sizeof(buffer));

    String topic = String(BUILDING_ID) + "/" + ROOM_ID + "/state";

    if (!mqttClient.connected()) {
      log("MQTT", "OFFLINE → KHONG GUI JSON");
      return;
    }

    bool ok = mqttClient.publish(topic.c_str(), buffer);
    log("MQTT", String("PUBLISH STATE → ") + (ok ? "OK" : "FAIL"));
  }

  static NetworkManager* instance;

private:
  bool ethernetReady;
  bool mqttConnected;
  uint32_t lastMqttReconnectAttempt;
  uint8_t mqttReconnectCount;
  unsigned long lastOutdoorReq;
  bool ntpSynced;

  RoomConfig &roomCfg;
  SensorState *sensorsRef;
  AcState *acsRef;
  AcManager &acManager;
  OledManager &oled;
  IrQueueManager &irQueueMgr;

  static void mqttCallbackStatic(char* topic, byte* payload, unsigned int length) {
    if (instance) instance->mqttCallback(topic, payload, length);
  }

  void setupEthernet() {
    log("ETH", "INIT...");
    Ethernet.init(PIN_ETH_CS);
    Ethernet.begin(mac);
    delay(1000);
    IPAddress ip = Ethernet.localIP();
    if (ip[0] == 0) {
      ethernetReady = false;
      log("ETH", "NO IP (0.0.0.0)");
    } else {
      ethernetReady = true;
      log("ETH", "IP = " + ip.toString());
    }
  }

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
    log("LWT", "ONLINE → " + key);
  }

  void mqttSubscribeAll() {
    mqttClient.subscribe((String(BUILDING_ID) + "/" + ROOM_ID + "/ac/+/power").c_str());
    mqttClient.subscribe((String(BUILDING_ID) + "/" + ROOM_ID + "/ac/+/mode").c_str());
    mqttClient.subscribe((String(BUILDING_ID) + "/" + ROOM_ID + "/ac/+/speed").c_str());
    mqttClient.subscribe((String(BUILDING_ID) + "/" + ROOM_ID + "/ac/+/swing").c_str());
    mqttClient.subscribe((String(BUILDING_ID) + "/" + ROOM_ID + "/ac/+/ctrl_temp").c_str());
    mqttClient.subscribe((String(BUILDING_ID) + "/" + ROOM_ID + "/ac/+/fac").c_str());

    String baseCfg = String(BUILDING_ID) + "/" + ROOM_ID + "/config/";
    mqttClient.subscribe((baseCfg + "temp_setauto").c_str());
    mqttClient.subscribe((baseCfg + "hum_setauto").c_str());
    mqttClient.subscribe((baseCfg + "thr_temp").c_str());

    String t_env = String(BUILDING_ID) + "/outdoor/sensor/1/temp";
    mqttClient.subscribe(t_env.c_str());
    log("MQTT SUB", t_env);
  }

  void mqttReconnect() {
    if (!ethernetReady) return;
    if (mqttClient.connected()) return;

    uint32_t now = millis();
    if (now - lastMqttReconnectAttempt < 8000) return;
    lastMqttReconnectAttempt = now;

    log("MQTT", "RECONNECTING...");

    String clientId = makeClientId();
    String lwtTopic = availabilityTopic();

    if (mqttClient.connect(clientId.c_str(),
                           mqtt_user, mqtt_pass,
                           lwtTopic.c_str(), 0, true, "offline")) {
      mqttConnected = true;
      mqttReconnectCount = 0;
      log("MQTT", "CONNECTED");
      publishAvailabilityOnline();
      if (ROLE_PICO == 2) mqttSubscribeAll();
      oled.wake();
    } else {
      mqttConnected = false;
      mqttReconnectCount++;
      log("MQTT", "FAIL (" + String(mqttReconnectCount) + ")");
    }
  }


  void handleEthernetLink() {
    if (Ethernet.linkStatus() == LinkOFF) {
      if (ethernetReady) {
        ethernetReady = false;
        log("ETH", "LINK OFF");
      }
    } else {
      if (!ethernetReady) {
        log("ETH", "LINK ON → REINIT");
        setupEthernet();
        oled.wake();
      }
    }
  }

  void handleMqtt() {
    bool prevConn = mqttConnected;
    //
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 3000) {   // 3 giây
        lastCheck = millis();
        if (!mqttClient.connected()) mqttReconnect();
    }

    //
    mqttClient.loop();
    mqttConnected = mqttClient.connected();
    if (!mqttConnected && prevConn) {
      log("MQTT", "DISCONNECTED SAU LOOP");
    }

    bool netStatus = mqttClient.connected();
    static bool lastNet = false;
    if (netStatus != lastNet) {
      lastNet = netStatus;
      log("NET CHANGE", netStatus ? "ONLINE" : "OFFLINE");
      oled.wake();
    }
  }

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
          log("NTP SYNC", timeClient.getFormattedTime());
        } else {
          log("NTP UPDATE", timeClient.getFormattedTime());
        }
      } else {
        log("NTP FAIL", "DNS/UDP CHUA SAN SANG");
      }
    }
  }

  void handleOutdoorRequest() {
    if (ROLE_PICO == 2 && mqttClient.connected()) {
      if (millis() - lastOutdoorReq > 60000) {
        String reqTopic = String(BUILDING_ID) + "/outdoor/request";
        mqttClient.publish(reqTopic.c_str(), "1");
        log("MQTT", "REQUEST OUTDOOR → " + reqTopic);
        lastOutdoorReq = millis();
      }
    }
  }

  void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String t = String(topic);
    String msg;
    for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

    log("MQTT RX", t + " = " + msg);

    int p1 = t.indexOf('/');
    int p2 = t.indexOf('/', p1 + 1);
    int p3 = t.indexOf('/', p2 + 1);
    int p4 = t.indexOf('/', p3 + 1);

    String lvl0 = (p1 > 0) ? t.substring(0, p1) : t;
    String lvl1 = (p1 > 0 && p2 > p1) ? t.substring(p1 + 1, p2) : "";
    String lvl2 = (p2 > 0 && p3 > p2) ? t.substring(p2 + 1, p3) : "";
    String lvl3 = (p3 > 0 && p4 > p3) ? t.substring(p3 + 1, p4) : "";
    String lvl4 = (p4 > 0) ? t.substring(p4 + 1) : "";

    if (lvl0 == BUILDING_ID && lvl1 == ROOM_ID && lvl2 == "ac") {
      int acIndex = lvl3.toInt() - 1;
      if (acIndex >= 0 && acIndex < NUM_AC) {
        acManager.applyUserCmd(acsRef[acIndex], lvl4, msg, acIndex);
        oled.wake();
      }
      return;
    }

    if (lvl0 == BUILDING_ID && lvl1 == ROOM_ID && lvl2 == "config") {
      acManager.updateConfig(lvl3, msg);
      oled.wake();
      return;
    }

    if (lvl0 == BUILDING_ID && lvl1 == "outdoor" && lvl2 == "sensor" && lvl3 == "1" && lvl4 == "temp") {
      acManager.setOutdoorTemp(msg.toFloat());
      return;
    }
  }

  static void log(const String &tag, const String &msg) {
    Serial.print("[");
    Serial.print(timeClient.getFormattedTime());
    Serial.print("] ");
    Serial.print(tag);
    Serial.print(": ");
    Serial.println(msg);
  }

  unsigned long lastNtpTry = 0;
};

NetworkManager* NetworkManager::instance = nullptr;

// ================== WATCHDOG ==================
void setupWatchdog() {
  watchdog_enable(8000, 1);
}

void feedWatchdog() {
  watchdog_update();
}

// ================== GLOBAL APP STATE ==================
RoomConfig roomCfg = {
  BUILDING_ID,
  ROOM_ID,
  27.0f,
  60.0f,
  24.0f
};

SensorState sensors[NUM_SENSOR];
AcState acs[NUM_AC];

IrQueueManager irQueueMgr;
OledManager oledMgr;
SensorManager sensorMgr;
AcManager acMgr(roomCfg, irQueueMgr);
NetworkManager netMgr(roomCfg, sensors, acs, acMgr, oledMgr, irQueueMgr);

// ================== PUBLISH SENSOR ERROR ==================
void publishSensorError() {
  String topic = String(BUILDING_ID) + "/" + ROOM_ID + "/sensor/error";
  String payload = "sensor_fail";
  if (mqttClient.connected()) {
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
  }
  Serial.print("[");
  Serial.print(timeClient.getFormattedTime());
  Serial.print("] ");
  Serial.print("ERROR SENSOR");
  Serial.print(": ");
  Serial.print("PUBLISH → ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(payload);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("===== PICO BMS OOP (NET + MQTT + NTP + IR SAFE, JSON STATE) =====");

  setupWatchdog();

  NetworkManager::instance = &netMgr;

  netMgr.setup();
  sensorMgr.setup();
  acMgr.setup(acs);
  oledMgr.setup();

  irsend.begin();
}

// ================== LOOP ==================
void loop() {
  feedWatchdog();

  static unsigned long lastBeat = 0;
  if (millis() - lastBeat > 2000) {
    lastBeat = millis();
    #if DEBUG
    Serial.print("[HB] loop alive, ms=");
    Serial.println(millis());
    #endif
  }

  netMgr.loop();

  if (ROLE_PICO == 1) {
    sensorMgr.readOutdoor(sensors);
  } else {
    sensorMgr.readIndoor(sensors);
  }

  SensorState* active = sensorMgr.getActiveSensor(sensors);

  if (ROLE_PICO == 2 && NUM_SENSOR == 1 && (!active || !active->ok)) {
    Serial.print("[");
    Serial.print(timeClient.getFormattedTime());
    Serial.print("] ");
    Serial.println("ERROR SENSOR: ssrom1 FAIL → DUNG AC");
    publishSensorError();
    for (int i = 0; i < NUM_AC; i++) {
      acs[i].power = "off";
    }
    oledMgr.update(netMgr.isMqttConnected(), active, netMgr.isNtpSynced());
    oledMgr.maybeSleep();
    delay(500);
    return;
  }

  if (active && !isnan(active->temp)) {
    oledMgr.onTempChangeWake(active->temp);
  }

  if (ROLE_PICO == 2) {
    acMgr.computeAuto(acs, active);
    irQueueMgr.process(acs, [](AcState &ac) {
      acMgr.sendIr(ac);
    });
  }

  // ================== IR ROUND ROBIN 10 GIÂY ==================
  static unsigned long lastRoundIr = 0;
  static int roundIndex = 0;

  if (millis() - lastRoundIr > 10000) {
    lastRoundIr = millis();

    if (acs[roundIndex].opr_mode == "auto" && acs[roundIndex].power == "on") {
      irQueueMgr.enqueue(roundIndex, false);
      Serial.print("[");
      Serial.print(timeClient.getFormattedTime());
      Serial.print("] IR ROUND ROBIN → ");
      Serial.println(acs[roundIndex].name);
    }

    roundIndex = (roundIndex + 1) % NUM_AC;
  }

  static uint32_t lastPub = 0;
  if (millis() - lastPub > 60000) {
    lastPub = millis();
    netMgr.publishRoomState();
  }

  oledMgr.update(netMgr.isMqttConnected(), active, netMgr.isNtpSynced());
  oledMgr.maybeSleep();
}
