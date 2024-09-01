#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "PWM/pwm.h"
#include <ADC/adc.h>

#include "driver/gpio.h" //gpio

#define PWM_DUTY_RES                10      //1023
#define PWM_PIN                     23      //GPIO23
#define PWM_FREQUENCY               10000

#define ADC_UNIT                    0       //ADC_UNIT_1
#define ADC_BITWIDTH                10      //ADC_BITWIDTH_10
#define ADC_ATTEN                   3       //ADC_ATTEN_DB_11
#define ADC_INPUT_VOLTAGE_PIN       5       //ADC_CHANNEL_5(GPIO33)
#define ADC_OUTPUT_VOLTAGE_PIN      4       //ADC_CHANNEL_5(GPIO32)

void app_main() 
{
    PWM_init(PWM_DUTY_RES, PWM_PIN, PWM_FREQUENCY);

    ADC1_init(ADC_UNIT, ADC_BITWIDTH, ADC_ATTEN);
    set_adc_pin(ADC_INPUT_VOLTAGE_PIN, ADC_OUTPUT_VOLTAGE_PIN);
   
    int adc_read1 = 0;
    int adc_read2 = 0;
    int voltage1 = 0;
    int voltage2 = 0;

    float duty_cycle = 0.0;

    while(1)
    {
        adc_read1 = adc_read_raw(ADC_INPUT_VOLTAGE_PIN);
        adc_read2 = adc_read_raw(ADC_OUTPUT_VOLTAGE_PIN);

        voltage1 = adc_read_voltage(ADC_INPUT_VOLTAGE_PIN);
        voltage2 = adc_read_voltage(ADC_OUTPUT_VOLTAGE_PIN);
         
        printf("ADCpin1 value = %d \n", adc_read1); 
        printf("ADCpin2 value = %d \n", adc_read2);

        printf("ADC value = %d mV\n", voltage1);
        printf("ADC value = %d mV\n", voltage2);

        PWM_duty_cycle(adc_read1);
        duty_cycle = ((float)adc_read1/1023.0) * 100.0;
        printf("Duty_cycle = %f %%\n ", duty_cycle); 

        vTaskDelay(10);
    }
}