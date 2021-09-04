#include <Arduino.h>

void IRAM_ATTR receive_transmission_interrupt();
void IRAM_ATTR update_encoder_right();
void IRAM_ATTR update_encoder_left();
void IRAM_ATTR sensor_ISR();