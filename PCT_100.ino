#include "exit.h"
#include "key.h"
#include "led.h"
#include "relay.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 10

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define I2C_SDA 4
#define I2C_SCL 5
U8G2_SH1106_128X64_VCOMH0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define LONG_PRESS_THRESHOLD 2000
#define LIGHT_PIN 1
#define LIGHT_THRESHOLD_LUX 300
#define TEMP_THRESHOLD_C 30

uint8_t auto_mode = 1;
float lastTempC = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== System Starting ===");
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
  Serial.println();
  Serial.println("Waiting for KEY1 to enable system...");
}

void loop() {
  handle_key1();
  handle_key2();
  update_outputs();
  delay(10);
}

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
        Serial.print("[KEY2] Short press - Manual State: ");
        Serial.print(function_mode);
        Serial.println(" (incremented)");
      }
    }
    key2_last = current;
  }
  
  if (current == 1 && !long_press_triggered && system_enabled) {
    unsigned long press_duration = millis() - key2_press_time;
    if (press_duration >= LONG_PRESS_THRESHOLD) {
      long_press_triggered = 1;
      auto_mode = !auto_mode;
      Serial.print("[KEY2] Long press detected: ");
      Serial.print(press_duration);
      Serial.println(" ms");
      if (auto_mode) {
        Serial.println(">>> Mode changed to: 切换到自动");
      } else {
        Serial.println(">>> Mode changed to: 切换到手动");
      }
    }
  }
}

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
        Serial.print("光敏ADC: "); Serial.print(lightVal);
        Serial.print(" | 温度: "); Serial.print(lastTempC, 1);
        Serial.print(" C|电压: "); Serial.print(lightV, 2); Serial.println(" V");
        u8g2.clearBuffer();
        u8g2.setCursor(0, 12); u8g2.print("模式:自动  总闸:开");
        u8g2.setCursor(0, 26); u8g2.print("光照: "); u8g2.print((int)lux); u8g2.print("/"); u8g2.print(LIGHT_THRESHOLD_LUX);
        u8g2.setCursor(0, 40); u8g2.print("温度: "); u8g2.print(lastTempC, 1); u8g2.print("/"); u8g2.print(TEMP_THRESHOLD_C);
        u8g2.setCursor(0, 54); u8g2.print("灯:");
        u8g2.setCursor(36, 54); u8g2.print(lux < LIGHT_THRESHOLD_LUX ? "开" : "关");
        u8g2.setCursor(72, 54); u8g2.print("风扇:");
        u8g2.setCursor(108, 54); u8g2.print(lastTempC > TEMP_THRESHOLD_C ? "开" : "关");
        u8g2.sendBuffer();
      }
      LED(lux < LIGHT_THRESHOLD_LUX ? HIGH : LOW);
      relay_control(lastTempC > TEMP_THRESHOLD_C ? HIGH : LOW);
    } else {
      static unsigned long lastDisp = 0;
      if (millis() - lastDisp >= 1000) {
        lastDisp = millis();
        u8g2.clearBuffer();
        u8g2.setCursor(0, 12); u8g2.print("模式:手动  总闸:开");
        {
          int v = analogRead(LIGHT_PIN);
          int inv = 4095 - v;
          float lx = (float)inv * inv / 30000.0;
          u8g2.setCursor(0, 26); u8g2.print("光照: "); u8g2.print((int)lx); u8g2.print("/"); u8g2.print(LIGHT_THRESHOLD_LUX);
        }
        u8g2.setCursor(0, 40); u8g2.print("温度: "); u8g2.print(lastTempC, 1); u8g2.print("/"); u8g2.print(TEMP_THRESHOLD_C);
        u8g2.setCursor(0, 54); u8g2.print("灯:");
        u8g2.setCursor(36, 54); u8g2.print(digitalRead(6) ? "开" : "关");
        u8g2.setCursor(72, 54); u8g2.print("风扇:");
        u8g2.setCursor(108, 54); u8g2.print(digitalRead(7) ? "开" : "关");
        u8g2.sendBuffer();
      }
      switch (function_mode) {
        case 0: {
          LED(LOW);
          relay_control(LOW);
          break;
        }
        case 1: {
          LED(LOW);
          relay_control(HIGH);
          break;
        }
        case 2: {
          LED(HIGH);
          relay_control(LOW);
          break;
        }
        case 3: {
          LED(HIGH);
          relay_control(HIGH);
          break;
        }
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
