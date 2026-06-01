#ifndef __LED_H
#define __LED_H

#include "Arduino.h"

#define LED_PIN 6

#define LED(x) digitalWrite(LED_PIN, x)
#define LED_TOGGLE() digitalWrite(LED_PIN, !digitalRead(LED_PIN))

void led_init(void);

#endif