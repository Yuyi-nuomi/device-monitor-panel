#ifndef __KEY_H
#define __KEY_H

#include "Arduino.h"

#define KEY1_PIN 20
#define KEY2_PIN 21

#define KEY1 digitalRead(KEY1_PIN)
#define KEY2 digitalRead(KEY2_PIN)

void key_init(void);

#endif