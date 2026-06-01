#include "exit.h"
#include "key.h"
#include "led.h"
#include "relay.h"

volatile uint8_t system_enabled = 0;
volatile uint8_t key1_pressed = 0;
uint8_t function_mode = 0;

void exit_init(void)
{
  key_init();
  led_init();
  relay_init();
  // 不使用中断，先纯轮询测试
}

void key1_isr(void)
{
  key1_pressed = 1;
}
