#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "PWM/pwm.h"

#include "driver/gpio.h" //gpio

#include "esp_adc/adc_oneshot.h" //adc
#include "hal/adc_types.h" //adc
#include "esp_adc/adc_cali.h" //adc

#define PWM_DUTY_RES    10 //1023
#define PWM_PIN         23
#define PWM_FREQUENCY   10000

void app_main() 
{
    PWM_init(PWM_DUTY_RES, PWM_PIN, PWM_FREQUENCY);

    int adc_read = 0;
    int voltage = 0;

    float duty_cycle = 0.0;

    adc_oneshot_unit_handle_t handle = NULL;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    adc_oneshot_new_unit(&init_config, &handle);

     adc_oneshot_chan_cfg_t config = {

        .bitwidth = ADC_BITWIDTH_10,
        .atten = ADC_ATTEN_DB_11,
    };

    adc_oneshot_config_channel(handle, ADC_CHANNEL_5, &config);

    adc_cali_handle_t cali_handle = NULL;

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_10,
    };

    adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);

    while(1)
    {
        adc_oneshot_read(handle, ADC_CHANNEL_5, &adc_read); 
        adc_cali_raw_to_voltage(cali_handle, adc_read, &voltage);
        printf("ADC value = %d mV\n", voltage); 

        PWM_duty_cycle(adc_read);

        duty_cycle = ((float)adc_read/1023.0) * 100.0;

        printf("Duty_cycle = %f\n", duty_cycle); 
        vTaskDelay(10);
    }
}