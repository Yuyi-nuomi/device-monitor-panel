#include "exit.h"
#include "key.h"
#include "led.h"
#include "relay.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>  // 必须加

// ====================== 在这里改你的 WiFi ======================
const char* WIFI_SSID     = "Pura 70";
const char* WIFI_PASSWORD = "WJK20230806lwx";
// ===============================================================

#define ONE_WIRE_BUS 10
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define I2C_SDA 4
#define I2C_SCL 5
U8G2_SH1106_128X64_VCOMH0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define LONG_PRESS_THRESHOLD 2000
#define LIGHT_PIN 1
#define LIGHT_THRESHOLD_LUX 300
#define LIGHT_HYSTERESIS 60   // 滞后量，防止频闪（减小以提高响应速度）
#define LIGHT_FILTER_SIZE 5    // 滤波窗口大小（减小以提高响应速度）
#define TEMP_THRESHOLD_C 30

uint8_t auto_mode = 1;
float lastTempC = 0;
static uint8_t led_state = LOW;  // LED当前状态，用于滞后控制

// 光照滤波相关
static int light_readings[LIGHT_FILTER_SIZE] = {0};  // 存储历史读数
static int light_reading_index = 0;  // 当前读数索引
static unsigned long last_light_update = 0;  // 上次更新光照的时间

// WiFi 状态变量
bool wifi_connected = false;
IPAddress local_IP;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== System Starting ===");
  Serial.println("Initializing system...");

  exit_init();
  sensors.begin();
  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  u8g2.setContrast(0x60);
  u8g2.setPowerSave(0);
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);
  u8g2.enableUTF8Print();
  delay(200);

  Serial.println("System initialization complete!");
  Serial.println("\n=== WiFi 自动连接 ===");

  // ========== 核心：开机自动连 WiFi ==========
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // 等待连接（最多10秒）
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    local_IP = WiFi.localIP();
    Serial.println("\nWiFi 连接成功！");
    Serial.print("IP地址: "); Serial.println(local_IP);
    Serial.print("网关: "); Serial.println(WiFi.gatewayIP());
    Serial.print("子网掩码: "); Serial.println(WiFi.subnetMask());
    Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  } else {
    wifi_connected = false;
    Serial.println("\nWiFi 连接失败！");
  }

  Serial.println("\nWaiting for KEY1 to enable system...");
}

void loop() {
  handle_key1();
  handle_key2();
  update_outputs();
  delay(10);
}

// 按键1：总开关
void handle_key1() {
  uint8_t current = KEY1;
  if (current == 1) {
    if (!system_enabled) {
      system_enabled = 1;
      auto_mode = 1;
      function_mode = 0;
      Serial.println("[KEY1] System ENABLED");
      Serial.println(">>> Mode: AUTO");
    }
  } else {
    if (system_enabled) {
      system_enabled = 0;
      Serial.println("[KEY1] System DISABLED");
    }
  }
}

// 按键2：长按切换自动/手动，短按切换手动状态
void handle_key2() {
  static uint8_t key2_last = 0;
  static unsigned long key2_press_time = 0;
  static uint8_t long_press_triggered = 0;

  uint8_t current = KEY2;

  if (current != key2_last) {
    if (current == 1) {
      key2_press_time = millis();
      long_press_triggered = 0;
    } else {
      if (!long_press_triggered && system_enabled && !auto_mode) {
        function_mode = (function_mode + 1) % 4;
        Serial.print("[KEY2] 短按 - 手动状态: ");
        Serial.println(function_mode);
      }
    }
    key2_last = current;
  }

  if (current == 1 && !long_press_triggered && system_enabled) {
    unsigned long press_duration = millis() - key2_press_time;
    if (press_duration >= LONG_PRESS_THRESHOLD) {
      long_press_triggered = 1;
      auto_mode = !auto_mode;
      if (auto_mode) Serial.println(">>> 切换到：自动");
      else Serial.println(">>> 切换到：手动");
    }
  }
}

// 输出控制 + OLED显示
void update_outputs() {
  if (system_enabled) {
    if (auto_mode) {
      int lightVal = analogRead(LIGHT_PIN);
      int invADC = 4095 - lightVal;
      float lux = (float)invADC * invADC / 30000.0;
      float lightV = lightVal * (3.3 / 4095.0);

      static unsigned long lastPrint = 0;
      if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        sensors.requestTemperatures();
        lastTempC = sensors.getTempCByIndex(0);

        Serial.print("光敏:"); Serial.print(lightVal);
        Serial.print(" | 温度:"); Serial.print(lastTempC,1);
        Serial.print(" | 电压:"); Serial.print(lightV,2); Serial.println("V");

        u8g2.clearBuffer();
        u8g2.setCursor(0,12); u8g2.print("模式:自动  总闸:开");
        u8g2.setCursor(0,26); u8g2.print("光照:"); u8g2.print((int)lux); u8g2.print("/"); u8g2.print(LIGHT_THRESHOLD_LUX);
        u8g2.setCursor(0,40); u8g2.print("温度:"); u8g2.print(lastTempC,1); u8g2.print("/"); u8g2.print(TEMP_THRESHOLD_C);

        // WiFi信息显示在OLED
        if (wifi_connected) {
          u8g2.setCursor(0,54); u8g2.print("WiFi:"); u8g2.print(local_IP.toString());
        } else {
          u8g2.setCursor(0,54); u8g2.print("WiFi:断开");
        }
        u8g2.sendBuffer();
      }

      // 数字滤波：每30ms更新一次读数，使用滑动平均
      static float filtered_lux = 0;
      if (millis() - last_light_update >= 30) {
        last_light_update = millis();
        
        // 存入当前读数
        light_readings[light_reading_index] = lightVal;
        light_reading_index = (light_reading_index + 1) % LIGHT_FILTER_SIZE;
        
        // 计算滑动平均值
        int sum = 0;
        for (int i = 0; i < LIGHT_FILTER_SIZE; i++) {
          sum += light_readings[i];
        }
        int avg_lightVal = sum / LIGHT_FILTER_SIZE;
        int avg_invADC = 4095 - avg_lightVal;
        filtered_lux = (float)avg_invADC * avg_invADC / 30000.0;
      }

      // 防频闪滞后控制（使用滤波后的值）
      if (led_state == LOW) {
        if (filtered_lux < LIGHT_THRESHOLD_LUX) {
          LED(HIGH);
          led_state = HIGH;
          Serial.println("[LED] 灯开启");
        }
      } else {
        if (filtered_lux > LIGHT_THRESHOLD_LUX + LIGHT_HYSTERESIS) {
          LED(LOW);
          led_state = LOW;
          Serial.println("[LED] 灯关闭");
        }
      }

      relay_control(lastTempC > TEMP_THRESHOLD_C ? HIGH : LOW);
    }
    else {
      static unsigned long lastDisp = 0;
      if (millis() - lastDisp >= 1000) {
        lastDisp = millis();
        u8g2.clearBuffer();
        u8g2.setCursor(0,12); u8g2.print("模式:手动  总闸:开");

        int v = analogRead(LIGHT_PIN);
        int inv = 4095 - v;
        float lx = (float)inv * inv / 30000.0;
        u8g2.setCursor(0,26); u8g2.print("光照:"); u8g2.print((int)lx); u8g2.print("/"); u8g2.print(LIGHT_THRESHOLD_LUX);
        u8g2.setCursor(0,40); u8g2.print("温度:"); u8g2.print(lastTempC,1); u8g2.print("/"); u8g2.print(TEMP_THRESHOLD_C);

        if (wifi_connected) {
          u8g2.setCursor(0,54); u8g2.print("WiFi:"); u8g2.print(local_IP.toString());
        } else {
          u8g2.setCursor(0,54); u8g2.print("WiFi:断开");
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
  }
  else {
    LED(LOW);
    relay_control(LOW);
    static unsigned long lastDisp = 0;
    if (millis() - lastDisp >= 1000) {
      lastDisp = millis();
      u8g2.clearBuffer();
      u8g2.setCursor(0,14); u8g2.print("总闸:关");
      u8g2.setCursor(0,40); u8g2.print("系统关闭");
      u8g2.sendBuffer();
    }
  }
}