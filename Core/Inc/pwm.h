
#include "main.h"
#ifndef PWM_H
#define PWM_H

void process_RC_channels(const crsf_header_t *p);
uint16_t get_pwm_value(uint8_t ch);

#endif //PWM_H
