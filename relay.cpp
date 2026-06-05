#include "relay.h"

static uint8_t relay_state = LOW;

void relay_init(void)
{
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  relay_state = LOW;
}

void relay_control(uint8_t state)
{
  digitalWrite(RELAY_PIN, state ? HIGH : LOW);
  relay_state = state;
}

uint8_t relay_get_state(void)
{
  return relay_state;
}