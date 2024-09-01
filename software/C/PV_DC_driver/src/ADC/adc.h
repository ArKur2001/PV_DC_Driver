#ifndef H_ADC
#define H_ADC

#include <inttypes.h>

void ADC1_init(uint8_t adc_unit, uint8_t adc_bitwidth, uint8_t adc_atten);
void set_adc_pin(uint8_t adc_pin1, uint8_t adc_pin2);
uint16_t adc_read_raw(uint8_t adc_pin);
uint16_t adc_read_voltage(uint8_t adc_pin);

#endif //ADC