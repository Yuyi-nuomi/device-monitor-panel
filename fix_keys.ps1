$enc = [System.Text.UTF8Encoding]::new($false)
$content = [System.IO.File]::ReadAllText([System.IO.Path]::GetFullPath("PCT_100.ino"), $enc)

# Fix 1: handle_key1 - last=0 -> last=1
$old1 = "  static uint8_t last = 0;"
$new1 = "  static uint8_t last = 1;  // KEY1 reads HIGH when not pressed"
$content = $content.Replace($old1, $new1)
Write-Host "1. handle_key1 last init fixed"

# Fix 2: handle_key2 - rewrite
$anchor = "void handle_key2()"
$start = $content.IndexOf($anchor)
$end = $content.IndexOf("`n", $content.IndexOf("void update_outputs", $start) - 1)
$oldFunc = $content.Substring($start, $content.IndexOf("void update_outputs", $start) - $start)

$newFunc = @"
void handle_key2() {
  static uint8_t last_key = 1;
  static unsigned long press_start = 0;
  static bool longPressTriggered = false;
  uint8_t curr = KEY2;

  // 按下时刻：开始计时
  if (curr == 0 && last_key == 1) {
    press_start = millis();
    longPressTriggered = false;
  }

  // 按住不放：检测长按
  if (curr == 0) {
    unsigned long hold = millis() - press_start;
    if (hold >= LONG_PRESS_THRESHOLD && !longPressTriggered && system_enabled) {
      longPressTriggered = true;
      auto_mode = !auto_mode;
      Serial.println(auto_mode ? ">>>切换自动模式" : ">>>切换手动模式");
      Serial.flush();
    }
    if (hold >= RESET_WIFI_LONG_PRESS && !longPressTriggered) {
      longPressTriggered = true;
      clearWiFi();
    }
  }

  // 松开时刻：判断是否为短按
  if (curr == 1 && last_key == 0) {
    unsigned long hold = millis() - press_start;
    if (hold > 30 && hold < LONG_PRESS_THRESHOLD && system_enabled && !auto_mode) {
      function_mode = (function_mode + 1) % 4;
      Serial.print("[KEY2] 手动档位：");
      Serial.println(function_mode);
      Serial.flush();
    }
  }

  last_key = curr;
}

"@

$content = $content.Remove($start, $oldFunc.Length)
$content = $content.Insert($start, $newFunc)
Write-Host "2. handle_key2 logic fixed"

# Fix 3: Remove delay(10) from loop
$content = $content.Replace("  delay(10);", "  // delay removed for responsiveness")
Write-Host "3. delay(10) removed from loop"

# Fix 4: key.cpp - INPUT -> INPUT_PULLUP
$keyCpp = [System.IO.File]::ReadAllText([System.IO.Path]::GetFullPath("key.cpp"), $enc)
$keyCpp = $keyCpp.Replace("pinMode(KEY1_PIN, INPUT);", "pinMode(KEY1_PIN, INPUT_PULLUP);")
$keyCpp = $keyCpp.Replace("pinMode(KEY2_PIN, INPUT);", "pinMode(KEY2_PIN, INPUT_PULLUP);")
[System.IO.File]::WriteAllText([System.IO.Path]::GetFullPath("key.cpp"), $keyCpp, $enc)
Write-Host "4. key.cpp: INPUT -> INPUT_PULLUP"

# Write main file
[System.IO.File]::WriteAllText([System.IO.Path]::GetFullPath("PCT_100.ino"), $content, $enc)
Write-Host "Done! PCT_100.ino written"
