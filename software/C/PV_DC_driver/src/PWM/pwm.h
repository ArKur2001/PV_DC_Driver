#ifndef H_PWM
#define H_PWM

#include <inttypes.h>

enum PWM_State {PWM_ON, PWM_OFF};

void PWM_init(uint8_t duty_resolution ,uint8_t output_pin ,uint32_t frequency);
void PWM_control(enum PWM_State ePWM_State_control);
enum PWM_State PWM_get_state();
void PWM_duty_cycle(uint16_t duty_cycle);

#endif //PWM