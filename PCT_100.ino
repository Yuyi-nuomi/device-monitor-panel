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

#define ONE_WIRE_BUS 10
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define I2C_SDA 4
#define I2C_SCL 5
U8G2_SH1106_128X64_VCOMH0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define LONG_PRESS_THRESHOLD 2000
#define RESET_WIFI_LONG_PRESS 5000  // 长按5秒重置WiFi

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

#define EEPROM_SIZE 512
#define EEPROM_FLAG_ADDR 0
#define EEPROM_SSID_ADDR 1
#define EEPROM_PWD_ADDR 33
const char WIFI_FLAG = 0xAB;

int rssiToPercent(int rssi) {
  if (rssi >= -50) return 100;
  if (rssi <= -100) return 0;
  return 2 * (rssi + 100);
}

void clearSerialBuffer() {
  while (Serial.available()) Serial.read();
}

// 保存WiFi
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

// 读取WiFi
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

// 清除WiFi（重置配网）
void clearWiFi() {
  EEPROM.write(EEPROM_FLAG_ADDR, 0x00);
  EEPROM.commit();
  Serial.println("🗑️ 已清除WiFi信息，重启重新配网");
  Serial.flush();
  delay(300);
  ESP.restart();
}

// 扫描配网
void startSmartConfig() {
  Serial.println("\n=====================================");
  Serial.println("        扫描WiFi");
  Serial.println("=====================================\n");
  Serial.flush();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int n = WiFi.scanNetworks();

  if (n == 0) {
    Serial.println("未找到WiFi");
    while (1) delay(100);
  }

  Serial.print("找到 "); Serial.print(n); Serial.println(" 个WiFi:");
  Serial.println("-------------------------------------");
  for (int i = 0; i < n; i++) {
    int rssi = WiFi.RSSI(i);
    int p = rssiToPercent(rssi);
    Serial.print(i+1); Serial.print(": ");
    Serial.print(WiFi.SSID(i));
    Serial.print(" ["); Serial.print(p); Serial.println("%]");
  }
  Serial.println("-------------------------------------");
  Serial.println("输入编号：");
  Serial.flush();

  while (!Serial.available()) delay(100);
  int sel = Serial.parseInt() - 1;
  clearSerialBuffer();

  if (sel >= 0 && sel < n) {
    targetSSID = WiFi.SSID(sel);
    Serial.print("选择: "); Serial.println(targetSSID);
    Serial.flush();
  } else {
    Serial.println("错误！重启");
    while (1) delay(100);
  }

  Serial.println("输入密码：");
  Serial.flush();
  while (!Serial.available()) delay(100);
  targetPWD = Serial.readStringUntil('\n');
  targetPWD.trim();
}

// 连接WiFi
bool connectWiFi() {
  Serial.print("连接: "); Serial.println(targetSSID);
  Serial.flush();
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
    return true;
  } else {
    wifi_connected = false;
    Serial.println("\n❌ 连接失败！");
    Serial.flush();
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
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

  // 优先读Flash
  if (loadWiFi()) {
    if (connectWiFi()) {
      // 成功
    } else {
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
  handle_key1();
  handle_key2();
  update_outputs();
  delay(10);
}

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
      Serial.println("[KEY1] 系统启动");
      Serial.flush();
    } else {
      system_enabled = 0;
      Serial.println("[KEY1] 系统关闭");
      Serial.flush();
    }
  }
}

void handle_key2() {
  static uint8_t last = 0;
  static unsigned long t_press = 0;
  static uint8_t long_flag = 0;

  uint8_t current = KEY2;
  if (current != last) {
    last = current;
    delay(20);
    if (current == 1) {
      t_press = millis();
      long_flag = 0;
    } else {
      if (!long_flag && system_enabled && !auto_mode) {
        function_mode = (function_mode + 1) % 4;
        Serial.print("[KEY2] 手动模式: "); Serial.println(function_mode);
        Serial.flush();
      }
    }
  }

  if (current == 1 && system_enabled) {
    unsigned long dur = millis() - t_press;

    // 长按5秒 → 重置WiFi
    if (dur >= RESET_WIFI_LONG_PRESS) {
      long_flag = 1;
      Serial.println("\n=====================================");
      Serial.println("     长按5秒：重置WiFi");
      Serial.println("=====================================\n");
      Serial.flush();
      clearWiFi(); // 清除并重启
    }

    // 原来的长按2秒切换自动/手动
    else if (dur >= LONG_PRESS_THRESHOLD && !long_flag) {
      long_flag = 1;
      auto_mode = !auto_mode;
      if (auto_mode) Serial.println(">>> 自动模式");
      else Serial.println(">>> 手动模式");
      Serial.flush();
    }
  }
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
        u8g2.setCursor(0, 12); u8g2.print("模式:自动  总闸:开");
        u8g2.setCursor(0, 26); u8g2.print("光照:"); u8g2.print((int)lux); u8g2.print("/"); u8g2.print(LIGHT_THRESHOLD_LUX);
        u8g2.setCursor(0, 40); u8g2.print("温度:"); u8g2.print(lastTempC, 1); u8g2.print("/"); u8g2.print(TEMP_THRESHOLD_C);
        if (wifi_connected) {
          u8g2.setCursor(0, 54); u8g2.print("WiFi:"); u8g2.print(local_IP.toString());
          u8g2.setCursor(105, 54); u8g2.print(rssiToPercent(WiFi.RSSI())); u8g2.print("%");
        } else {
          u8g2.setCursor(0, 54); u8g2.print("WiFi:未连接");
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
        u8g2.setCursor(0, 12); u8g2.print("模式:手动  总闸:开");
        int v = analogRead(LIGHT_PIN);
        int inv = 4095 - v;
        float lx = (float)inv * inv / 30000.0;
        u8g2.setCursor(0, 26); u8g2.print("光照:"); u8g2.print((int)lx); u8g2.print("/"); u8g2.print(LIGHT_THRESHOLD_LUX);
        u8g2.setCursor(0, 40); u8g2.print("温度:"); u8g2.print(lastTempC, 1); u8g2.print("/"); u8g2.print(TEMP_THRESHOLD_C);
        if (wifi_connected) {
          u8g2.setCursor(0, 54); u8g2.print("WiFi:"); u8g2.print(local_IP.toString());
          u8g2.setCursor(105, 54); u8g2.print(rssiToPercent(WiFi.RSSI())); u8g2.print("%");
        } else {
          u8g2.setCursor(0, 54); u8g2.print("WiFi:未连接");
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
      u8g2.setCursor(0, 14); u8g2.print("总闸:关");
      u8g2.setCursor(0, 40); u8g2.print("系统关闭");
      u8g2.sendBuffer();
    }
  }
}