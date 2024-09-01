#ifndef H_PWM
#define H_PWM

#include <inttypes.h>

void PWM_init(uint8_t duty_resolution ,uint8_t output_pin ,uint32_t frequency);
void PWM_duty_cycle(uint16_t duty_cycle);

#endif // PWM