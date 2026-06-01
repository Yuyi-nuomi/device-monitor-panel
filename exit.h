#ifndef __EXIT_H
#define __EXIT_H

#include "Arduino.h"

extern volatile uint8_t system_enabled;
extern volatile uint8_t key1_pressed;
extern uint8_t function_mode;

void exit_init(void);
void key1_isr(void);

#endif