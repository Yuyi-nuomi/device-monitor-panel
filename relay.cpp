#include "relay.h"

void relay_init(void)
{
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
}

void relay_control(uint8_t state)
{
  digitalWrite(RELAY_PIN, state ? HIGH : LOW);
}