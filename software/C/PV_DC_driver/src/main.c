#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h" //gpio


#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_cali.h"

void app_main() 
{
    int adc_read =0;
    int voltage = 0;

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
        printf("ADC value = %d mv\n", voltage); 
        vTaskDelay(100);
    }
}