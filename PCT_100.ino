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
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "secrets.h"

// ==================== 硬件引脚配置 ====================
#define WS2812_PIN 0
#define LED_NUM 1
CRGB rgbLed[LED_NUM];

#define ONE_WIRE_BUS 10
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define I2C_SDA 4
#define I2C_SCL 5
U8G2_SH1106_128X64_VCOMH0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define LONG_PRESS_THRESHOLD 2000
#define RESET_WIFI_LONG_PRESS 5000
#define LIGHT_PIN 1
#define LIGHT_HYSTERESIS 60
#define LIGHT_FILTER_SIZE 5

// 阈值全局变量
int LIGHT_THRESHOLD_LUX = 300;
float TEMP_THRESHOLD_C = 30.0;

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

unsigned long wifiReconnectTimer = 0;
const unsigned long RECONNECT_INTERVAL = 3000;
const unsigned long SHOW_FAIL_TIME = 5000;
unsigned long disconnectStartDisp = 0;

#define EEPROM_SIZE 512
#define EEPROM_FLAG_ADDR 0
#define EEPROM_SSID_ADDR 1
#define EEPROM_PWD_ADDR 33
const char WIFI_FLAG = 0xAB;

// ==================== 多彩灯全局变量 ====================
bool mqtt_ok = true;
unsigned long rgb_run_timer = 0;
uint8_t rgb_work_mode = 0;
uint8_t rgb_step = 0;

uint8_t grad_sr=0, grad_sg=0, grad_sb=0;
uint8_t grad_er=0, grad_eg=0, grad_eb=0;
unsigned long grad_start_time=0;
uint16_t grad_dur=0;
bool is_gradual = false;

// ==================== FreeRTOS 多线程配置【核心】====================
#define TASK_STACK_SIZE 2048
#define TASK_PRIORITY 1

TaskHandle_t wifiMqttTaskHandle = NULL;
TaskHandle_t rgbLedTaskHandle = NULL;

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
bool needPublish = false;
// ======================================================================

int rssiToPercent(int rssi) {
  if (rssi >= -60) return 100;
  if (rssi <= -90) return 0;
  return map(rssi, -90, -60, 0, 100);
}

void clearSerialBuffer() {
  while (Serial.available()) Serial.read();
}

// ==================== RGB 基础函数 ====================
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  rgbLed[0] = CRGB(r, g, b);
  FastLED.show();
}

void rgbBootBlink() {
  for (int i = 0; i < 3; i++) {
    setRGB(255, 0, 0); vTaskDelay(pdMS_TO_TICKS(300));
    setRGB(0, 255, 0); vTaskDelay(pdMS_TO_TICKS(300));
  }
  setRGB(0, 0, 0);
}

void rgbGradualStart(uint16_t dur, uint8_t sr, uint8_t sg, uint8_t sb, uint8_t er, uint8_t eg, uint8_t eb) {
  grad_sr = sr; grad_sg = sg; grad_sb = sb;
  grad_er = er; grad_eg = eg; grad_eb = eb;
  grad_dur = dur;
  grad_start_time = millis();
  is_gradual = true;
}

bool rgbGradualUpdate() {
  if(!is_gradual) return true;
  unsigned long elapsed = millis() - grad_start_time;
  if(elapsed >= grad_dur) {
    setRGB(grad_er, grad_eg, grad_eb);
    is_gradual = false;
    return true;
  }
  float ratio = (float)elapsed / grad_dur;
  uint8_t r = grad_sr + (grad_er - grad_sr) * ratio;
  uint8_t g = grad_sg + (grad_eg - grad_sg) * ratio;
  uint8_t b = grad_sb + (grad_eb - grad_sb) * ratio;
  setRGB(r, g, b);
  return false;
}

// ==================== 多彩灯任务函数 【线程2】====================
void rgbLedTask(void *pvParameters)
{
  for(;;)
  {
    bool fan_state, led_on_flag, light_alarm;
    portENTER_CRITICAL(&dataMux);
    fan_state = (lastTempC > TEMP_THRESHOLD_C);
    led_on_flag = (led_state == HIGH);
    int lightVal = analogRead(LIGHT_PIN);
    float lux = (float)(4095 - lightVal) * (4095 - lightVal) / 30000.0f;
    light_alarm = (lux < LIGHT_THRESHOLD_LUX);
    portEXIT_CRITICAL(&dataMux);

    static uint8_t last_mode = 0;
    uint8_t new_mode = 0;

    if(system_enabled){
      if(led_on_flag && fan_state){
        new_mode = 1;
      }else if(led_on_flag && !fan_state && light_alarm){
        new_mode = 2;
      }else if(!led_on_flag && fan_state){
        new_mode = 3;
      }else if(!mqtt_ok){
        new_mode = 4;
      }else if(!wifi_connected){
        new_mode = 5;
      }else{
        new_mode = 6;
      }
    }else{
      setRGB(0,0,0);
      rgb_step=0;
      rgb_run_timer=millis();
      is_gradual=false;
      last_mode=0;
      rgb_work_mode=0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if(new_mode != last_mode){
      rgb_step = 0;
      rgb_run_timer = millis();
      is_gradual = false;
      rgb_work_mode = new_mode;
      last_mode = new_mode;
    }

    unsigned long now = millis();
    if(is_gradual){
      if(rgbGradualUpdate()){
        rgb_step++;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    switch(rgb_work_mode){
      case 1:
        switch(rgb_step){
          case 0: setRGB(255,0,0); rgb_run_timer=now; rgb_step++; break;
          case 1: if(now-rgb_run_timer>=100){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 2: if(now-rgb_run_timer>=50){setRGB(255,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 3: if(now-rgb_run_timer>=100){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 4: if(now-rgb_run_timer>=50){setRGB(0,255,0); rgb_run_timer=now; rgb_step++;} break;
          case 5: if(now-rgb_run_timer>=100){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 6: if(now-rgb_run_timer>=50){setRGB(0,255,0); rgb_run_timer=now; rgb_step++;} break;
          case 7: if(now-rgb_run_timer>=100){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 8: if(now-rgb_run_timer>=50){setRGB(0,0,255); rgb_run_timer=now; rgb_step++;} break;
          case 9: if(now-rgb_run_timer>=100){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 10:if(now-rgb_run_timer>=50){setRGB(0,0,255); rgb_run_timer=now; rgb_step++;} break;
          case 11:if(now-rgb_run_timer>=100){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 12:if(now-rgb_run_timer>=350){rgb_step=0; rgb_run_timer=now;} break;
        }
        break;
      case 2:
        switch(rgb_step){
          case 0: setRGB(255,0,0); rgb_run_timer=now; rgb_step++; break;
          case 1: if(now-rgb_run_timer>=300){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 2: if(now-rgb_run_timer>=200){setRGB(0,255,0); rgb_run_timer=now; rgb_step++;} break;
          case 3: if(now-rgb_run_timer>=300){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 4: if(now-rgb_run_timer>=200){setRGB(0,0,255); rgb_run_timer=now; rgb_step++;} break;
          case 5: if(now-rgb_run_timer>=300){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 6: if(now-rgb_run_timer>=200){rgb_step=0; rgb_run_timer=now;} break;
        }
        break;
      case 3:
        switch(rgb_step){
          case 0: setRGB(255,0,0); rgb_run_timer=now; rgb_step++; break;
          case 1: if(now-rgb_run_timer>=500){setRGB(0,255,0); rgb_run_timer=now; rgb_step++;} break;
          case 2: if(now-rgb_run_timer>=500){setRGB(0,0,255); rgb_run_timer=now; rgb_step++;} break;
          case 3: if(now-rgb_run_timer>=500){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 4: if(now-rgb_run_timer>=700){rgb_step=0; rgb_run_timer=now;} break;
        }
        break;
      case 4:
        switch(rgb_step){
          case 0: setRGB(255,0,0); rgbGradualStart(500,255,0,0,0,255,0); rgb_step++; break;
          case 1: if(rgbGradualUpdate()){rgbGradualStart(500,0,255,0,0,0,0); rgb_step++;} break;
          case 2: if(rgbGradualUpdate()){rgbGradualStart(500,0,0,0,0,0,255); rgb_step++;} break;
          case 3: if(rgbGradualUpdate()){rgbGradualStart(500,0,0,255,0,0,0); rgb_step++;} break;
          case 4: if(rgbGradualUpdate()){rgb_step=0;} break;
        }
        break;
      case 5:
        switch(rgb_step){
          case 0: setRGB(255,0,0); rgb_run_timer=now; rgb_step++; break;
          case 1: if(now-rgb_run_timer>=200){setRGB(0,255,0); rgb_run_timer=now; rgb_step++;} break;
          case 2: if(now-rgb_run_timer>=200){setRGB(0,0,0); rgb_run_timer=now; rgb_step++;} break;
          case 3: if(now-rgb_run_timer>=500){rgb_step=0; rgb_run_timer=now;} break;
        }
        break;
      case 6:
        switch(rgb_step){
          case 0: setRGB(0,0,0); rgbGradualStart(1000,0,0,0,255,0,0); rgb_step++; break;
          case 1: if(rgbGradualUpdate()){rgb_step++;} break;
          case 2: rgbGradualStart(1000,255,0,0,0,255,0); rgb_step++; break;
          case 3: if(rgbGradualUpdate()){rgb_step++;} break;
          case 4: rgbGradualStart(1000,0,255,0,0,0,255); rgb_step++; break;
          case 5: if(rgbGradualUpdate()){rgb_step++;} break;
          case 6: rgbGradualStart(1000,0,0,255,0,0,0); rgb_step++; break;
          case 7: if(rgbGradualUpdate()){rgb_run_timer=now; rgb_step++;} break;
          case 8: if(now-rgb_run_timer>=200){rgb_step=0;} break;
        }
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== MQTT 回调函数 ====================
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences prefs;

#define MQTT_MODE_EXTERNAL 0
#define MQTT_MODE_INTERNAL 1
String mqttServer = String(MQTT_SERVER);
int mqttPort = MQTT_PORT;
String mqttUser = String(MQTT_USER);
String mqttPass = String(MQTT_PASS);
String deviceId = String(DEVICE_ID);
int mqttMode = MQTT_MODE_EXTERNAL;
String pubTopic, subTopic;

unsigned long lastMqttReconnect = 0;
const unsigned long mqttReconnectDelay = 5000;
unsigned long lastPublish = 0;
const unsigned long publishInterval = 2000;

void loadMqttConfig() {
  prefs.begin("mqtt", true);
  mqttServer = prefs.getString("ip", mqttServer);
  mqttPort   = prefs.getInt("port", mqttPort);
  mqttUser   = prefs.getString("user", mqttUser);
  mqttPass   = prefs.getString("pass", mqttPass);
  deviceId   = prefs.getString("id", deviceId);
  mqttMode   = prefs.getInt("mode", mqttMode);

  TEMP_THRESHOLD_C = prefs.getFloat("temp_th", 30.0);
  LIGHT_THRESHOLD_LUX = prefs.getInt("light_th", 300);
  prefs.end();
  pubTopic = "chemctrl/" + deviceId + "/status";
  subTopic = "chemctrl/" + deviceId + "/command";
}

void saveMqttConfig() {
  prefs.begin("mqtt", false);
  prefs.putString("ip", mqttServer);
  prefs.putInt("port", mqttPort);
  prefs.putString("user", mqttUser);
  prefs.putString("pass", mqttPass);
  prefs.putString("id", deviceId);
  prefs.putInt("mode", mqttMode);
  prefs.putFloat("temp_th", TEMP_THRESHOLD_C);
  prefs.putInt("light_th", LIGHT_THRESHOLD_LUX);
  prefs.end();
}

void publishStatus() {
  if (!mqttClient.connected() || !wifi_connected) return;
  StaticJsonDocument<256> doc;
  portENTER_CRITICAL(&dataMux);
  doc["temperature"] = (int)(lastTempC * 10) / 10.0;
  doc["light"] = analogRead(LIGHT_PIN);
  doc["mode"] = auto_mode ? "auto" : "manual";
  doc["key1_lock"] = system_enabled ? true : false;
  uint8_t light_state = (function_mode == 2 || function_mode == 3) ? 1 : 0;
  uint8_t fan_state   = (function_mode == 1 || function_mode == 3) ? 1 : 0;
  doc["relay3"] = light_state ? true : false;
  doc["relay4"] = fan_state ? true : false;
  doc["temp_threshold"] = TEMP_THRESHOLD_C;
  doc["light_threshold"] = LIGHT_THRESHOLD_LUX;
  portEXIT_CRITICAL(&dataMux);

  char buf[256];
  serializeJson(doc, buf);
  mqttClient.publish(pubTopic.c_str(), buf);
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, len)) return;
  String debug_msg; serializeJson(doc, debug_msg);
  Serial.println("\n[MQTT cmd] " + debug_msg);
  const char* cmd = doc["cmd"];

  if (strcmp(cmd, "get_status") == 0) {
    needPublish = true;
  }
  else if (strcmp(cmd, "set_relay") == 0) {
    if (!system_enabled || auto_mode) return;
    int relay = doc["relay"]; bool val = doc["value"];
    portENTER_CRITICAL(&dataMux);
    if (relay == 3) {
      if (val) function_mode |= 0x02; else function_mode &= ~0x02;
    } else if (relay == 4) {
      if (val) function_mode |= 0x01; else function_mode &= ~0x01;
    }
    portEXIT_CRITICAL(&dataMux);
    needPublish = true;
  }
  else if (strcmp(cmd, "set_mode") == 0) {
    if (!system_enabled) return;
    const char* mode = doc["mode"];
    portENTER_CRITICAL(&dataMux);
    if (strcmp(mode, "auto") == 0) auto_mode = true;
    else if (strcmp(mode, "manual") == 0) auto_mode = false;
    portEXIT_CRITICAL(&dataMux);
    needPublish = true;
  }
  else if (strcmp(cmd, "set_threshold") == 0) {
    if (doc.containsKey("temp")) {
      float t = doc["temp"];
      if (t > 0 && t < 100) {
        portENTER_CRITICAL(&dataMux);
        TEMP_THRESHOLD_C = t;
        portEXIT_CRITICAL(&dataMux);
        Serial.println("✅ 温度阈值已更新：" + String(TEMP_THRESHOLD_C));
      }
    }
    if (doc.containsKey("light")) {
      int l = doc["light"];
      if (l > 0 && l < 1000) {
        portENTER_CRITICAL(&dataMux);
        LIGHT_THRESHOLD_LUX = l;
        portEXIT_CRITICAL(&dataMux);
        Serial.println("✅ 光照阈值已更新：" + String(LIGHT_THRESHOLD_LUX));
      }
    }
    saveMqttConfig();
    needPublish = true;
  }
}

// ==================== WiFi+MQTT 任务函数 【线程1】====================
void wifiMqttTask(void *pvParameters)
{
  for(;;)
  {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck >= 1000)
    {
      lastCheck = millis();
      if (WiFi.status() == WL_CONNECTED)
      {
        wifi_connected = true;
        disconnectStartDisp = 0;
        wifiReconnectTimer = 0;
      }
      else
      {
        if(disconnectStartDisp == 0) disconnectStartDisp = millis();
        wifi_connected = false;
        if(targetSSID!="" && millis()-wifiReconnectTimer>RECONNECT_INTERVAL)
        {
          wifiReconnectTimer = millis();
          Serial.println("WiFi掉线，尝试自动重连...");
          WiFi.disconnect();
          WiFi.begin(targetSSID.c_str(),targetPWD.c_str());
        }
      }
    }

    mqtt_ok = mqttClient.connected() && wifi_connected;
    if (wifi_connected)
    {
      if (mqttClient.connected())
      {
        mqttClient.loop();
        if(needPublish)
        {
          publishStatus();
          needPublish = false;
        }
        static unsigned long lastMqttLog = 0;
        if (millis() - lastMqttLog >= 30000)
        {
          lastMqttLog = millis();
          Serial.print("[MQTT] connected to ");
          Serial.print(mqttServer);
          Serial.print(":");
          Serial.println(mqttPort);
        }
      }
      else
      {
        if (millis() - lastMqttReconnect < mqttReconnectDelay)
        {
          vTaskDelay(pdMS_TO_TICKS(100));
          continue;
        }
        lastMqttReconnect = millis();
        Serial.print("[MQTT] Connecting to ");
        Serial.print(mqttServer);
        Serial.print(":");
        Serial.println(mqttPort);

        String clientId = "ESP32_" + deviceId + "_" + String(random(0xffff), HEX);
        if (mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPass.c_str()))
        {
          mqttClient.subscribe(subTopic.c_str());
          publishStatus();
          Serial.println("[MQTT] Connected! Device: " + deviceId);
        }
        else
        {
          int st = mqttClient.state();
          Serial.print("- MQTT failed: ");
          Serial.println(st);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ==================== WiFi 工具函数 ====================
void saveWiFi(String ssid, String pwd) {
  EEPROM.write(EEPROM_FLAG_ADDR, WIFI_FLAG);
  for (int i = 0; i < ssid.length(); i++) EEPROM.write(EEPROM_SSID_ADDR + i, ssid[i]);
  EEPROM.write(EEPROM_SSID_ADDR + ssid.length(), '\0');
  for (int i = 0; i < pwd.length(); i++) EEPROM.write(EEPROM_PWD_ADDR + i, pwd[i]);
  EEPROM.write(EEPROM_PWD_ADDR + pwd.length(), '\0');
  EEPROM.commit();
  Serial.println("✅ WiFi已保存到Flash");
}

bool loadWiFi() {
  if (EEPROM.read(EEPROM_FLAG_ADDR) != WIFI_FLAG) {
    Serial.println("ℹ️ 无保存WiFi，进入配网");
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
  return true;
}

void clearWiFi() {
  EEPROM.write(EEPROM_FLAG_ADDR, 0x00);
  EEPROM.commit();
  Serial.println("\n🗑️ 已清除WiFi信息，设备重启重新配网！");
  vTaskDelay(pdMS_TO_TICKS(800));
  ESP.restart();
}

void startSmartConfig() {
  Serial.println("\n=====================================");
  Serial.println("        扫描附近WiFi列表");
  Serial.println("=====================================\n");

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
    while (1) vTaskDelay(pdMS_TO_TICKS(100));
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

  u8g2.clearBuffer();
  u8g2.setCursor(0, 28);
  u8g2.print("扫描完成");
  u8g2.setCursor(0, 48);
  u8g2.print("串口输入编号");
  u8g2.sendBuffer();

  while (!Serial.available()) vTaskDelay(pdMS_TO_TICKS(100));
  int sel = Serial.parseInt() - 1;
  clearSerialBuffer();

  if (sel >= 0 && sel < n) {
    targetSSID = WiFi.SSID(sel);
    Serial.print("已选择: "); Serial.println(targetSSID);
  } else {
    Serial.println("编号错误，重启设备");
    u8g2.clearBuffer();
    u8g2.setCursor(0,28);
    u8g2.print("编号错误");
    u8g2.sendBuffer();
    while (1) vTaskDelay(pdMS_TO_TICKS(100));
  }

  Serial.println("请输入WiFi密码：");
  u8g2.clearBuffer();
  u8g2.setCursor(0, 28);
  u8g2.print("等待输入密码");
  u8g2.sendBuffer();

  while (!Serial.available()) vTaskDelay(pdMS_TO_TICKS(100));
  targetPWD = Serial.readStringUntil('\n');
  targetPWD.trim();
}

bool connectWiFi() {
  Serial.print("正在连接: "); Serial.println(targetSSID);

  u8g2.clearBuffer();
  u8g2.setCursor(0, 28);
  u8g2.print("WiFi连接中...");
  u8g2.sendBuffer();

  WiFi.begin(targetSSID.c_str(), targetPWD.c_str());
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 25) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
    retry++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    local_IP = WiFi.localIP();
    Serial.println("\n✅ 连接成功！");
    Serial.print("IP: "); Serial.println(local_IP);
    Serial.print("信号: "); Serial.print(rssiToPercent(WiFi.RSSI())); Serial.println("%");

    u8g2.clearBuffer();
    u8g2.setCursor(0,28);
    u8g2.print("WiFi连接成功");
    u8g2.sendBuffer();
    vTaskDelay(pdMS_TO_TICKS(800));
    return true;
  } else {
    wifi_connected = false;
    Serial.println("\n❌ 连接失败！");
    u8g2.clearBuffer();
    u8g2.setCursor(0,28);
    u8g2.print("WiFi连接失败");
    u8g2.sendBuffer();
    vTaskDelay(pdMS_TO_TICKS(1200));
    return false;
  }
}

// ==================== 串口指令解析 ====================
void parseSerialMqtt() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.startsWith("#")) return;
  bool configChanged = false;
  if (line.startsWith("#IP:"))   { mqttServer = line.substring(4); configChanged = true; }
  if (line.startsWith("#PORT:")) { mqttPort = line.substring(6).toInt(); configChanged = true; }
  if (line.startsWith("#USER:")) { mqttUser = line.substring(6); configChanged = true; }
  if (line.startsWith("#PASS:")) { mqttPass = line.substring(6); configChanged = true; }
  if (line.startsWith("#ID:"))   { deviceId = line.substring(4); configChanged = true; }
  if (line.startsWith("#MODE:INTERNAL")) { mqttMode=MQTT_MODE_INTERNAL; mqttPort=1883; configChanged=true; Serial.println("-> 内网模式(1883)"); }
  if (line.startsWith("#MODE:EXTERNAL")) { mqttMode=MQTT_MODE_EXTERNAL; mqttPort=8081; configChanged=true; Serial.println("-> 外网模式(8081)"); }
  if (line.startsWith("#HELP")) {
    Serial.println();
    Serial.println("===== MQTT Commands =====");
    Serial.println("#IP:<addr>  #PORT:<port>  #USER:<user>  #PASS:<pass>  #ID:<id>");
    Serial.println("#MODE:INTERNAL | #MODE:EXTERNAL | #STATUS | #HELP");
    return;
  }
  if (line.startsWith("#STATUS")) {
    Serial.println();
    Serial.println("===== MQTT Status =====");
    Serial.print("Server: "); Serial.println(mqttServer);
    Serial.print("Port: "); Serial.println(mqttPort);
    Serial.print("User: "); Serial.println(mqttUser);
    Serial.print("Device: "); Serial.println(deviceId);
    Serial.print("Mode: "); Serial.println(mqttMode==MQTT_MODE_INTERNAL?"Internal":"External");
    Serial.print("WiFi: "); Serial.println(WiFi.status()==WL_CONNECTED?"Connected":"Disconnected");
    Serial.print("MQTT: "); Serial.println(mqttClient.connected()?"Connected":"Disconnected");
    return;
  }
  if (!configChanged) return;
  pubTopic = "chemctrl/" + deviceId + "/status";
  subTopic = "chemctrl/" + deviceId + "/command";
  saveMqttConfig();
  mqttClient.setServer(mqttServer.c_str(), mqttPort);
  if (mqttClient.connected()) mqttClient.disconnect();
  lastMqttReconnect = 0;
  Serial.println("+ MQTT config saved, reconnecting...");
}

// ==================== 按键处理 ====================
void handle_key1() {
  uint8_t current = KEY1;
  static uint8_t last = 0;
  if (current != last) {
    last = current;
    vTaskDelay(pdMS_TO_TICKS(20));
    if (current == 1) {
      system_enabled = 1;
      auto_mode = 1;
      function_mode = 0;
      Serial.println("[KEY1] 系统已启动");
      needPublish = true;
    } else {
      system_enabled = 0;
      Serial.println("[KEY1] 系统已关闭");
      needPublish = true;
    }
  }
}

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
      portENTER_CRITICAL(&dataMux);
      auto_mode = !auto_mode;
      portEXIT_CRITICAL(&dataMux);
      Serial.println(auto_mode ? ">>>切换自动模式" : ">>>切换手动模式");
      needPublish = true;
    }
    if (hold >= RESET_WIFI_LONG_PRESS) {
      clearWiFi();
    }
  }

  if (curr == 0 && last_key == 1) {
    unsigned long hold = millis() - press_start;
    if (hold < LONG_PRESS_THRESHOLD && system_enabled && !auto_mode) {
      portENTER_CRITICAL(&dataMux);
      function_mode = (function_mode + 1) % 4;
      portEXIT_CRITICAL(&dataMux);
      Serial.print("[KEY2] 手动档位：");
      Serial.println(function_mode);
      needPublish = true;
    }
  }
  last_key = curr;
}

// ==================== 传感器+继电器+OLED ====================
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
        float newTemp = sensors.getTempCByIndex(0);

        portENTER_CRITICAL(&dataMux);
        bool tempOver = (newTemp > TEMP_THRESHOLD_C) != (lastTempC > TEMP_THRESHOLD_C);
        lastTempC = newTemp;
        portEXIT_CRITICAL(&dataMux);

        static float oldLux = lux;
        bool lightOver = (lux < LIGHT_THRESHOLD_LUX) != (oldLux < LIGHT_THRESHOLD_LUX);
        oldLux = lux;

        if(tempOver || lightOver) needPublish = true;

        Serial.print("光敏:"); Serial.print(lightVal);
        Serial.print(" 温度:"); Serial.print(lastTempC, 1);
        Serial.println("");

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
        if (filtered_lux < LIGHT_THRESHOLD_LUX) { LED(HIGH); led_state = HIGH; needPublish = true; }
      } else {
        if (filtered_lux > LIGHT_THRESHOLD_LUX + LIGHT_HYSTERESIS) { LED(LOW); led_state = LOW; needPublish = true; }
      }

      bool fanNow = (lastTempC > TEMP_THRESHOLD_C);
      relay_control(fanNow ? HIGH : LOW);
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
          case 0: strcpy(lampStr,"OFF");strcpy(fanStr,"OFF");break;
          case 1: strcpy(lampStr,"OFF");strcpy(fanStr,"ON");break;
          case 2: strcpy(lampStr,"ON");strcpy(fanStr,"OFF");break;
          case 3: strcpy(lampStr,"ON");strcpy(fanStr,"ON");break;
        }
        u8g2.print("灯光:");u8g2.print(lampStr);
        u8g2.setCursor(62,46);
        u8g2.print("风扇:");u8g2.print(fanStr);
        u8g2.setCursor(0,58);
        if (wifi_connected) {
          u8g2.print("WiFi:"); u8g2.print(local_IP.toString());
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

// ==================== 初始化 & 主线程入口 ====================
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  FastLED.addLeds<WS2812,WS2812_PIN,GRB>(rgbLed, LED_NUM);
  FastLED.clear();
  vTaskDelay(pdMS_TO_TICKS(500));

  Serial.println("\n=====================================");
  Serial.println("          系统启动（多线程版）");
  Serial.println("=====================================\n");

  exit_init();
  sensors.begin();
  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  u8g2.setContrast(0x60);
  u8g2.setPowerSave(0);
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);
  u8g2.enableUTF8Print();
  vTaskDelay(pdMS_TO_TICKS(200));

  rgbBootBlink();

  loadMqttConfig();
  mqttClient.setServer(mqttServer.c_str(), mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(15);

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

  xTaskCreate(
    wifiMqttTask,
    "WiFi_MQTT_Task",
    TASK_STACK_SIZE,
    NULL,
    TASK_PRIORITY,
    &wifiMqttTaskHandle
  );

  xTaskCreate(
    rgbLedTask,
    "RGB_Led_Task",
    TASK_STACK_SIZE,
    NULL,
    TASK_PRIORITY,
    &rgbLedTaskHandle
  );

  Serial.println("\n等待 KEY1 启动系统...");
}

void loop() {
  handle_key1();
  handle_key2();
  update_outputs();
  parseSerialMqtt();
  vTaskDelay(pdMS_TO_TICKS(10));
}