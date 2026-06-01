#ifndef __RELAY_H
#define __RELAY_H

#include "Arduino.h"

#define RELAY_PIN 7

void relay_init(void);
void relay_control(uint8_t state);

#endif