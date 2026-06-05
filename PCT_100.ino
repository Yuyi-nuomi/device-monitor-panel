#include "exit.h"
#include "key.h"
#include "led.h"
#include "relay.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <FastLED.h>

// ====================== 【你原来的全部代码 100% 不动】 ======================
// ========== WS2812配置 ==========
#define WS2812_PIN 0
#define LED_NUM 1
CRGB rgbLed[LED_NUM];
// ===============================

#define ONE_WIRE_BUS 10
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define I2C_SDA 4
#define I2C_SCL 5
U8G2_SH1106_128X64_VCOMH0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define LONG_PRESS_THRESHOLD 2000
#define RESET_WIFI_LONG_PRESS 5000

#define LIGHT_PIN 1
#define LIGHT_THRESHOLD_LUX 300
#define LIGHT_HYSTERESIS 60
#define LIGHT_FILTER_SIZE 5
#define TEMP_THRESHOLD_C 30

uint8_t auto_mode = 1;
float lastTempC = 0;
static uint8_t led_state = LOW;

static int light_readings[LIGHT_FILTER_SIZE] = {0};
static int light_reading_index = 0;
static unsigned long last_light_update = 0;

bool wifi_connected = false;
IPAddress local_IP;
String targetSSID = "";
String targetPWD = "";

//WiFi重连参数
unsigned long wifiReconnectTimer = 0;
const unsigned long RECONNECT_INTERVAL = 3000;    //3秒重试
const unsigned long SHOW_FAIL_TIME = 5000;       //5秒后显示失败
unsigned long disconnectStartDisp = 0;    //全局掉线计时

#define EEPROM_SIZE 512
#define EEPROM_FLAG_ADDR 0
#define EEPROM_SSID_ADDR 1
#define EEPROM_PWD_ADDR 33
const char WIFI_FLAG = 0xAB;

int rssiToPercent(int rssi) {
  if (rssi >= -60) return 100;
  if (rssi <= -90) return 0;
  return map(rssi, -90, -60, 0, 100);
}

void clearSerialBuffer() {
  while (Serial.available()) Serial.read();
}

// ========== RGB 函数 ==========
void setRGB(uint8_t r, uint8_t b, uint8_t g) {
  rgbLed[0] = CRGB(r, g, b);
  FastLED.show();
}

// 开机红绿闪烁3次 → 熄灭
void rgbBootBlink() {
  for (int i = 0; i < 3; i++) {
    setRGB(255, 0, 0); delay(300);
    setRGB(0, 255, 0); delay(300);
  }
  setRGB(0, 0, 0);
}

// WiFi 实时状态检测+分级RGB【绿/蓝/黄/红】+自动重连
void checkWiFiStatus() {
  static unsigned long lastCheck = 0;
  static unsigned long disconnectStart = 0;

  if (millis() - lastCheck >= 1000) {
    lastCheck = millis();

    if (WiFi.status() == WL_CONNECTED) {
      wifi_connected = true;
      disconnectStart = 0;
      disconnectStartDisp = 0;
      wifiReconnectTimer = 0;
      int rssiPer = rssiToPercent(WiFi.RSSI());
      if(rssiPer >70){
        setRGB(0,255,0);    //强信号：绿
      }else if(rssiPer>30){
        setRGB(0,0,255);    //中信号：蓝
      }else{
        setRGB(255,255,0);  //弱信号：黄
      }
    } else {
      if(disconnectStart == 0){
        disconnectStart = millis();
        disconnectStartDisp = millis();
      }
      wifi_connected = false;
      setRGB(255, 0, 0);
      if(targetSSID!="" && millis()-wifiReconnectTimer>RECONNECT_INTERVAL){
        wifiReconnectTimer = millis();
        Serial.println("WiFi掉线，尝试自动重连...");
        WiFi.disconnect();
        WiFi.begin(targetSSID.c_str(),targetPWD.c_str());
      }
    }
  }
}
// ===============================

void saveWiFi(String ssid, String pwd) {
  EEPROM.write(EEPROM_FLAG_ADDR, WIFI_FLAG);
  for (int i = 0; i < ssid.length(); i++) EEPROM.write(EEPROM_SSID_ADDR + i, ssid[i]);
  EEPROM.write(EEPROM_SSID_ADDR + ssid.length(), '\0');
  for (int i = 0; i < pwd.length(); i++) EEPROM.write(EEPROM_PWD_ADDR + i, pwd[i]);
  EEPROM.write(EEPROM_PWD_ADDR + pwd.length(), '\0');
  EEPROM.commit();
  Serial.println("✅ WiFi已保存到Flash");
  Serial.flush();
}

bool loadWiFi() {
  if (EEPROM.read(EEPROM_FLAG_ADDR) != WIFI_FLAG) {
    Serial.println("ℹ️ 无保存WiFi，进入配网");
    Serial.flush();
    return false;
  }
  char ssid[32] = {0};
  for (int i = 0; i < 32; i++) ssid[i] = EEPROM.read(EEPROM_SSID_ADDR + i);
  targetSSID = String(ssid);
  char pwd[64] = {0};
  for (int i = 0; i < 64; i++) pwd[i] = EEPROM.read(EEPROM_PWD_ADDR + i);
  targetPWD = String(pwd);
  Serial.println("✅ 读取保存的WiFi");
  Serial.print("WiFi: "); Serial.println(targetSSID);
  Serial.flush();
  return true;
}

void clearWiFi() {
  EEPROM.write(EEPROM_FLAG_ADDR, 0x00);
  EEPROM.commit();
  Serial.println("\n🗑️ 已清除WiFi信息，设备重启重新配网！");
  Serial.flush();
  delay(800);
  ESP.restart();
}

void startSmartConfig() {
  Serial.println("\n=====================================");
  Serial.println("        扫描附近WiFi列表");
  Serial.println("=====================================\n");
  Serial.flush();

  // 配网时OLED先亮：正在扫描
  u8g2.clearBuffer();
  u8g2.setCursor(0, 28);
  u8g2.print("正在扫描WiFi...");
  u8g2.setCursor(0, 48);
  u8g2.print("请稍候...");
  u8g2.sendBuffer();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int n = WiFi.scanNetworks();

  if (n == 0) {
    Serial.println("未搜索到任何WiFi");
    u8g2.clearBuffer();
    u8g2.setCursor(0,28);
    u8g2.print("未找到WiFi");
    u8g2.sendBuffer();
    while (1) delay(100);
  }

  Serial.print("共找到 "); Serial.print(n); Serial.println(" 个WiFi:");
  Serial.println("-------------------------------------");
  for (int i = 0; i < n; i++) {
    int rssi = WiFi.RSSI(i);
    int p = rssiToPercent(rssi);
    Serial.print(i+1); Serial.print(": ");
    Serial.print(WiFi.SSID(i));
    Serial.print(" [信号"); Serial.print(p); Serial.println("%]");
  }
  Serial.println("-------------------------------------");
  Serial.println("请输入WiFi编号：");
  Serial.flush();

  // 扫描完成，提示串口输入
  u8g2.clearBuffer();
  u8g2.setCursor(0, 28);
  u8g2.print("扫描完成");
  u8g2.setCursor(0, 48);
  u8g2.print("串口输入编号");
  u8g2.sendBuffer();

  while (!Serial.available()) delay(100);
  int sel = Serial.parseInt() - 1;
  clearSerialBuffer();

  if (sel >= 0 && sel < n) {
    targetSSID = WiFi.SSID(sel);
    Serial.print("已选择: "); Serial.println(targetSSID);
    Serial.flush();
  } else {
    Serial.println("编号错误，重启设备");
    u8g2.clearBuffer();
    u8g2.setCursor(0,28);
    u8g2.print("编号错误");
    u8g2.sendBuffer();
    while (1) delay(100);
  }

  Serial.println("请输入WiFi密码：");
  Serial.flush();

  u8g2.clearBuffer();
  u8g2.setCursor(0, 28);
  u8g2.print("等待输入密码");
  u8g2.sendBuffer();

  while (!Serial.available()) delay(100);
  targetPWD = Serial.readStringUntil('\n');
  targetPWD.trim();
}

bool connectWiFi() {
  Serial.print("正在连接: "); Serial.println(targetSSID);
  Serial.flush();

  // 连接中界面
  u8g2.clearBuffer();
  u8g2.setCursor(0, 28);
  u8g2.print("WiFi连接中...");
  u8g2.sendBuffer();

  WiFi.begin(targetSSID.c_str(), targetPWD.c_str());
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 25) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    local_IP = WiFi.localIP();
    Serial.println("\n✅ 连接成功！");
    Serial.print("IP: "); Serial.println(local_IP);
    Serial.print("信号: "); Serial.print(rssiToPercent(WiFi.RSSI())); Serial.println("%");
    Serial.flush();

    u8g2.clearBuffer();
    u8g2.setCursor(0,28);
    u8g2.print("WiFi连接成功");
    u8g2.sendBuffer();
    delay(800);
    return true;
  } else {
    wifi_connected = false;
    Serial.println("\n❌ 连接失败！");
    Serial.flush();

    u8g2.clearBuffer();
    u8g2.setCursor(0,28);
    u8g2.print("WiFi连接失败");
    u8g2.sendBuffer();
    delay(1200);
    return false;
  }
}

// ====================== 【你原来的按键 & 输出逻辑 100% 不动】 ======================
void handle_key1();
void handle_key2();
void update_outputs();

// ==================================================================================
// ====================== 【只修复报错 + 固定8081端口】 =================
// ==================================================================================
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences prefs;

// 【固定你的 8081 端口，不改了！】
String mqttServer = "47.98.170.18";
int mqttPort = 8081;
String mqttUser = "dzdx_emqx";
String mqttPass = "Jp4!sQ7$";
String deviceId = "PCT_100_28";
String pubTopic, subTopic;

unsigned long lastMqttReconnect = 0;
const unsigned long mqttReconnectDelay = 5000;
unsigned long lastPublish = 0;
const unsigned long publishInterval = 2000;

// 【修复编译报错：声明函数】
uint8_t relay_get_state();

void loadMqttConfig() {
  prefs.begin("mqtt", true);
  mqttServer = prefs.getString("svr", mqttServer);
  mqttPort   = prefs.getInt("port", mqttPort);
  mqttUser   = prefs.getString("user", mqttUser);
  mqttPass   = prefs.getString("pass", mqttPass);
  deviceId   = prefs.getString("id", deviceId);
  prefs.end();
  pubTopic = "chemctrl/" + deviceId + "/status";
  subTopic = "chemctrl/" + deviceId + "/command";
}

void saveMqttConfig() {
  prefs.begin("mqtt", false);
  prefs.putString("svr", mqttServer);
  prefs.putInt("port", mqttPort);
  prefs.putString("user", mqttUser);
  prefs.putString("pass", mqttPass);
  prefs.putString("id", deviceId);
  prefs.end();
}

void parseSerialMqtt() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.startsWith("#")) return;

  if (line.startsWith("#IP:"))   mqttServer = line.substring(4);
  if (line.startsWith("#PORT:")) mqttPort = line.substring(6).toInt();
  if (line.startsWith("#USER:")) mqttUser = line.substring(6);
  if (line.startsWith("#PASS:")) mqttPass = line.substring(6);
  if (line.startsWith("#ID:"))   deviceId = line.substring(4);

  pubTopic = "chemctrl/" + deviceId + "/status";
  subTopic = "chemctrl/" + deviceId + "/command";
  saveMqttConfig();
  if (mqttClient.connected()) mqttClient.disconnect();
  mqttClient.setServer(mqttServer.c_str(), mqttPort);
}

void publishStatus() {
  if (!mqttClient.connected() || !wifi_connected) return;
  if (millis() - lastPublish < publishInterval) return;
  lastPublish = millis();

  StaticJsonDocument<192> doc;
  doc["device"] = deviceId;
  doc["enable"] = system_enabled;
  doc["auto"] = auto_mode;
  doc["temp"] = lastTempC;
  doc["light"] = analogRead(LIGHT_PIN);
  doc["led"] = led_state;
  // 【修复：直接读取继电器状态】
  doc["fan"] = digitalRead(12); // 改成你实际继电器引脚即可
  doc["light_th"] = LIGHT_THRESHOLD_LUX;
  doc["temp_th"] = TEMP_THRESHOLD_C;

  char buf[200];
  serializeJson(doc, buf);
  mqttClient.publish(pubTopic.c_str(), buf);
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  payload[len] = 0;
  StaticJsonDocument<192> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) return;

  if (doc.containsKey("enable")) system_enabled = doc["enable"];
  if (doc.containsKey("auto"))   auto_mode  = doc["auto"];
  if (doc.containsKey("led"))    { led_state = doc["led"]; LED(led_state); }
  if (doc.containsKey("fan"))    { relay_control(doc["fan"]); }
}

void mqttLoop() {
  if (!wifi_connected) return;
  if (mqttClient.connected()) {
    mqttClient.loop();
    publishStatus();
    return;
  }
  if (millis() - lastMqttReconnect < mqttReconnectDelay) return;
  lastMqttReconnect = millis();

  String clientId = "ESP32_" + deviceId + "_" + String(random(1000));
  mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPass.c_str());
  mqttClient.subscribe(subTopic.c_str());

  Serial.print("MQTT 尝试连接: ");
  Serial.print(mqttServer);
  Serial.print(":");
  Serial.println(mqttPort);
}

// ==================================================================================
// ====================== 【新增结束】 ======================
// ==================================================================================

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  FastLED.addLeds<WS2812,WS2812_PIN,GRB>(rgbLed, LED_NUM);
  FastLED.clear();
  delay(500);
  Serial.println("\n=====================================");
  Serial.println("          系统启动");
  Serial.println("=====================================\n");
  Serial.flush();

  exit_init();
  sensors.begin();
  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  u8g2.setContrast(0x60);
  u8g2.setPowerSave(0);
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);
  u8g2.enableUTF8Print();
  delay(200);

  rgbBootBlink();

  // === 加载MQTT配置 ===
  loadMqttConfig();
  mqttClient.setServer(mqttServer.c_str(), mqttPort);
  mqttClient.setCallback(mqttCallback);

  if (loadWiFi()) {
    if (!connectWiFi()) {
      Serial.println("⚠️ 原有WiFi失效，进入配网");
      startSmartConfig();
      if (connectWiFi()) saveWiFi(targetSSID, targetPWD);
    }
  } else {
    startSmartConfig();
    if (connectWiFi()) saveWiFi(targetSSID, targetPWD);
  }

  Serial.println("\n等待 KEY1 启动系统...");
  Serial.flush();
}

void loop() {
  parseSerialMqtt();
  handle_key1();
  handle_key2();
  update_outputs();
  checkWiFiStatus();
  mqttLoop();
  delay(10);
}

// ====================== 【你原来的完整函数 100% 不动】 ======================
void handle_key1() {
  uint8_t current = KEY1;
  static uint8_t last = 0;
  if (current != last) {
    last = current;
    delay(20);
    if (current == 1) {
      system_enabled = 1;
      auto_mode = 1;
      function_mode = 0;
      Serial.println("[KEY1] 系统已启动");
      Serial.flush();
    } else {
      system_enabled = 0;
      Serial.println("[KEY1] 系统已关闭");
      Serial.flush();
    }
  }
}

// ====================== 已修改：按住2秒立即触发，无需松手 ======================
void handle_key2() {
  static uint8_t last_key = 0;
  static unsigned long press_start = 0;
  static bool longPressTriggered = false;
  uint8_t curr = KEY2;

  if (curr == 1 && last_key == 0) {
    press_start = millis();
    longPressTriggered = false;
  }

  if (curr == 1) {
    unsigned long hold = millis() - press_start;

    if (hold >= LONG_PRESS_THRESHOLD && !longPressTriggered && system_enabled) {
      longPressTriggered = true;
      auto_mode = !auto_mode;
      Serial.println(auto_mode ? ">>>切换自动模式" : ">>>切换手动模式");
      Serial.flush();
    }

    if (hold >= RESET_WIFI_LONG_PRESS) {
      clearWiFi();
    }
  }

  if (curr == 0 && last_key == 1) {
    unsigned long hold = millis() - press_start;
    if (hold < LONG_PRESS_THRESHOLD && system_enabled && !auto_mode) {
      function_mode = (function_mode + 1) % 4;
      Serial.print("[KEY2] 手动档位：");
      Serial.println(function_mode);
      Serial.flush();
    }
  }

  last_key = curr;
}

void update_outputs() {
  if (system_enabled) {
    if (auto_mode) {
      int lightVal = analogRead(LIGHT_PIN);
      int invADC = 4095 - lightVal;
      float lux = (float)invADC * invADC / 30000.0;
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        sensors.requestTemperatures();
        lastTempC = sensors.getTempCByIndex(0);
        Serial.print("光敏:"); Serial.print(lightVal);
        Serial.print(" 温度:"); Serial.print(lastTempC, 1);
        Serial.println("");
        Serial.flush();

        u8g2.clearBuffer();
        u8g2.setCursor(0,10);
        u8g2.print(auto_mode?"模式:自动  总闸:ON":"模式:手动  总闸:ON");
        u8g2.setCursor(0,22);
        u8g2.print("光照:");u8g2.print((int)lux);u8g2.print(" / ");u8g2.print(LIGHT_THRESHOLD_LUX);
        u8g2.setCursor(0,34);
        u8g2.print("温度:");u8g2.print(lastTempC,1);u8g2.print(" / ");u8g2.print(TEMP_THRESHOLD_C);
        u8g2.setCursor(0,46);
        u8g2.print("灯光:");u8g2.print(led_state?"ON":"OFF");
        u8g2.setCursor(62,46);
        u8g2.print("风扇:");u8g2.print((lastTempC>TEMP_THRESHOLD_C)?"ON":"OFF");
        u8g2.setCursor(0,58);
        if(wifi_connected){
          u8g2.print("WiFi:");u8g2.print(local_IP.toString());
          u8g2.print("   ");
          u8g2.setCursor(105,58);u8g2.print(rssiToPercent(WiFi.RSSI()));u8g2.print("%");
        }else{
          if(millis()-disconnectStartDisp < SHOW_FAIL_TIME){
            u8g2.print("WiFi:重连中...");
          }else{
            u8g2.print("WiFi:重连失败");
          }
        }
        u8g2.sendBuffer();
      }

      static float filtered_lux = 0;
      if (millis() - last_light_update >= 30) {
        last_light_update = millis();
        light_readings[light_reading_index] = lightVal;
        light_reading_index = (light_reading_index + 1) % LIGHT_FILTER_SIZE;
        int sum = 0;
        for (int i = 0; i < LIGHT_FILTER_SIZE; i++) sum += light_readings[i];
        int avg_lightVal = sum / LIGHT_FILTER_SIZE;
        filtered_lux = (float)(4095 - avg_lightVal) * (4095 - avg_lightVal) / 30000.0;
      }

      if (led_state == LOW) {
        if (filtered_lux < LIGHT_THRESHOLD_LUX) { LED(HIGH); led_state = HIGH; }
      } else {
        if (filtered_lux > LIGHT_THRESHOLD_LUX + LIGHT_HYSTERESIS) { LED(LOW); led_state = LOW; }
      }
      relay_control(lastTempC > TEMP_THRESHOLD_C ? HIGH : LOW);
    }
    else
    {
      static unsigned long lastDisp = 0;
      if (millis() - lastDisp >= 1000) {
        lastDisp = millis();
        u8g2.clearBuffer();
        u8g2.setCursor(0,10);
        u8g2.print("模式:手动  总闸:ON");
        int v = analogRead(LIGHT_PIN);
        int inv = 4095 - v;
        float lx = (float)inv * inv / 30000.0;
        u8g2.setCursor(0,22);
        u8g2.print("光照:");u8g2.print((int)lx);u8g2.print(" / ");u8g2.print(LIGHT_THRESHOLD_LUX);
        u8g2.setCursor(0,34);
        u8g2.print("温度:");u8g2.print(lastTempC,1);u8g2.print(" / ");u8g2.print(TEMP_THRESHOLD_C);
        u8g2.setCursor(0,46);
        char lampStr[5],fanStr[5];
        switch(function_mode){
          case 0: strcpy(lampStr,"OFF");strcpy(fanStr,"OFF"); break;
          case 1: strcpy(lampStr,"OFF");strcpy(fanStr,"ON");  break;
          case 2: strcpy(lampStr,"ON"); strcpy(fanStr,"OFF"); break;
          case 3: strcpy(lampStr,"ON"); strcpy(fanStr,"ON");  break;
        }
        u8g2.print("灯光:");u8g2.print(lampStr);
        u8g2.setCursor(62,46);
        u8g2.print("风扇:");u8g2.print(fanStr);
        u8g2.setCursor(0,58);
        if (wifi_connected) {
          u8g2.print("WiFi:"); u8g2.print(local_IP.toString());
          u8g2.print("   ");
          u8g2.setCursor(105,58); u8g2.print(rssiToPercent(WiFi.RSSI())); u8g2.print("%");
        } else {
          if(millis()-disconnectStartDisp < SHOW_FAIL_TIME){
            u8g2.print("WiFi:重连中...");
          }else{
            u8g2.print("WiFi:重连失败");
          }
        }
        u8g2.sendBuffer();
      }

      switch (function_mode) {
        case 0: LED(LOW);  relay_control(LOW);  break;
        case 1: LED(LOW);  relay_control(HIGH); break;
        case 2: LED(HIGH); relay_control(LOW);  break;
        case 3: LED(HIGH); relay_control(HIGH); break;
      }
    }
  } else {
    LED(LOW);
    relay_control(LOW);
    static unsigned long lastDisp = 0;
    if (millis() - lastDisp >= 1000) {
      lastDisp = millis();
      u8g2.clearBuffer();
      u8g2.setCursor(0,22); u8g2.print("总闸:OFF 系统关闭");
      u8g2.sendBuffer();
    }
  }
}