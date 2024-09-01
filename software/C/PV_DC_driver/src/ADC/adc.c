#include <ADC/adc.h>
#include "esp_adc/adc_oneshot.h" 
#include "hal/adc_types.h" 
#include "esp_adc/adc_cali.h" 

uint8_t bitwidth = 0;
uint8_t atten = 0;
adc_oneshot_unit_handle_t adc1_handle;  
adc_cali_handle_t adc1_cali_handle;     

void ADC1_init(uint8_t adc_unit, uint8_t adc_bitwidth, uint8_t adc_atten)
{
    bitwidth = adc_bitwidth;
    atten = adc_atten;

    adc_oneshot_unit_init_cfg_t adc_1_init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    adc_oneshot_new_unit(&adc_1_init_config, &adc1_handle);  
        
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = atten,
        .bitwidth = adc_bitwidth,
    };

    adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle);
}

void set_adc_pin(uint8_t adc_pin1, uint8_t adc_pin2)
{
    adc_oneshot_chan_cfg_t adc1_pin_cfg = {
        .bitwidth = bitwidth,
        .atten = atten,
    };

   adc_oneshot_config_channel(adc1_handle, adc_pin1, &adc1_pin_cfg); //ADC_INPUT_VOLTAGE_PIN
   adc_oneshot_config_channel(adc1_handle, adc_pin2, &adc1_pin_cfg); //ADC_OUTPUT_VOLTAGE_PIN
}

uint16_t adc_read_raw(uint8_t adc_pin)
{
    uint16_t adc_input_voltage_read = 0;

    adc_oneshot_read(adc1_handle, adc_pin, &adc_input_voltage_read);
    
    return adc_input_voltage_read;
}

uint16_t adc_read_voltage(uint8_t adc_pin)
{
    uint16_t input_voltage = 0;

    adc_cali_raw_to_voltage(adc1_cali_handle, adc_read_raw(adc_pin), &input_voltage);

    return input_voltage;
}
