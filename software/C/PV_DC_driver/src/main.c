#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h" //gpio

#include "esp_adc/adc_oneshot.h" //adc
#include "hal/adc_types.h" //adc
#include "esp_adc/adc_cali.h" //adc

#include "driver/ledc.h"//PWM

#define PWM_TIMER              LEDC_TIMER_1
#define PWM_MODE               LEDC_HIGH_SPEED_MODE
#define PWM_CLK                LEDC_AUTO_CLK
#define PWM_OUTPUT_IO          23
#define PWM_CHANNEL            LEDC_CHANNEL_0
#define PWM_DUTY_RES           LEDC_TIMER_10_BIT 
#define PWM_DUTY               512 
#define PWM_FREQUENCY          10000 

static void PWM_init(void)
{
    ledc_timer_config_t pwm_timer = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_DUTY_RES,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQUENCY,  
        .clk_cfg          = PWM_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    ledc_channel_config_t pwm_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .intr_type      = 0,
        .gpio_num       = PWM_OUTPUT_IO,
        .duty           = 0, 
        .hpoint         = 0
    };
    ledc_channel_config(&pwm_channel);
}

void app_main() 
{
    PWM_init();

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

        ledc_set_duty(PWM_MODE, PWM_CHANNEL, adc_read);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL);

        duty_cycle = ((float)adc_read/1023.0) * 100.0;

        printf("Duty_cycle = %f\n", duty_cycle); 
        vTaskDelay(10);
    }
}