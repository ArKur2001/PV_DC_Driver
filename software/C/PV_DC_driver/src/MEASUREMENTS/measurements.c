#include <MEASUREMENTS/measurements.h>
#include <math.h>

uint16_t voltage_reference_level = 0;
float voltage_multiplier_val = 0.0;
uint16_t current_reference_level = 0;

void measurements_init(uint16_t voltage_v_ref, float voltage_multiplier, uint16_t current_v_ref)
{
    voltage_reference_level = 2 * voltage_v_ref;
    voltage_multiplier_val = voltage_multiplier;
    current_reference_level = 2 * current_v_ref;
}

double get_voltage_value(int voltage, float duty_value, float pwm_duty_resolution)
{
    double duty_cycle = 0.0;
    double RMS_voltage_value = 0.0;

    duty_cycle = duty_value / pwm_duty_resolution;

    RMS_voltage_value = (((voltage_reference_level - voltage) * voltage_multiplier_val) * (sqrt(duty_cycle))) / 1000.0;

    return RMS_voltage_value;
}

double get_current_value(int voltage, float duty_value, float pwm_duty_resolution)
{
    double duty_cycle = 0.0;
    double RMS_current_value = 0.0;

    duty_cycle = duty_value / pwm_duty_resolution;

    RMS_current_value = ((current_reference_level - voltage) / (sqrt(duty_cycle))) / 100.0;

    return RMS_current_value;
}
