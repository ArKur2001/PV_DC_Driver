#ifndef H_MEASUREMENTS
#define H_MEASUREMENTS

#include <inttypes.h>

void measurements_init(uint16_t voltage_v_ref, uint16_t voltage_multiplier, uint16_t current_v_ref);
double get_voltage_value(int voltage, float duty_value, float pwm_duty_resolution);
double get_current_value(int voltage, float duty_value, float pwm_duty_resolution);

#endif //H_MEASUREMENTS