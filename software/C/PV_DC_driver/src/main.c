#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "PWM/pwm.h"
#include <ADC/adc.h>

#include "driver/gpio.h" //gpio
#include "esp_rom_sys.h"

#define PWM_DUTY_RES                10      //1023
#define PWM_PIN                     23      //GPIO23
#define PWM_FREQUENCY               10000

#define ADC_UNIT                    0       //ADC_UNIT_1
#define ADC_BITWIDTH                10      //ADC_BITWIDTH_10
#define ADC_ATTEN                   3       //ADC_ATTEN_DB_11
#define ADC_INPUT_VOLTAGE_PIN       5       //ADC_CHANNEL_5(GPIO33)
#define ADC_OUTPUT_VOLTAGE_PIN      4       //ADC_CHANNEL_5(GPIO32)

float get_current_value(void)
{
    u_int8_t sample_counter = 0;
    int buffer = 0;
    float result = 0.0;

    for(sample_counter = 0 ; sample_counter < 100 ; sample_counter++)
    {
        if(gpio_get_level(GPIO_NUM_22) == 1)
        {
            buffer = buffer + adc_read_voltage(ADC_INPUT_VOLTAGE_PIN);
        }
        else
        {
           sample_counter--; 
        }

        esp_rom_delay_us(10); 
    }

    result = buffer/100.0;
    result = (result - 2500.0)*-1.0;
    result = result * 0.707;
    result = result / 1000;

    return result;

}

void app_main() 
{
    printf("START1 \n");

    PWM_init(PWM_DUTY_RES, PWM_PIN, PWM_FREQUENCY);

    printf("START2 \n");

    ADC1_init(ADC_UNIT, ADC_BITWIDTH, ADC_ATTEN);
    set_adc_pin(ADC_INPUT_VOLTAGE_PIN, ADC_OUTPUT_VOLTAGE_PIN);
   
    //int voltage1 = 0;
    
    printf("START3 \n");
    gpio_set_direction(GPIO_NUM_22, GPIO_MODE_INPUT);

    while(1)
    {
        //voltage1 = adc_read_voltage(ADC_INPUT_VOLTAGE_PIN);
        
        //printf("ADC value = %d mV\n", voltage1);
       
        printf("START4 \n");

        PWM_duty_cycle(512);

        printf("RMS value = %f \n", get_current_value());

        vTaskDelay(100);
    }
}