#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "PWM/pwm.h"
#include <ADC/adc.h>
#include "BUTTONS/buttons.h"

#include "driver/gpio.h" //gpio
#include "esp_rom_sys.h"
#include <math.h>

#define PWM_DUTY_RES_BIT            7       //128
#define PWM_DUTY_RES                128
#define PWM_PIN                     23      //GPIO23
#define PWM_FREQUENCY               10000

#define ADC_UNIT                    0       //ADC_UNIT_1
#define ADC_BITWIDTH                12      //ADC_BITWIDTH_12
#define ADC_ATTEN                   3       //ADC_ATTEN_DB_11
#define ADC_SAMPLES_NUMBER          100     
#define ADC_CURRENT_PIN             5       //ADC_CHANNEL_5(GPIO33)
#define ADC_VOLTAGE_PIN             4       //ADC_CHANNEL_5(GPIO32)

#define BUTTON_0_GPIO               27      //GPIO23     
#define BUTTON_1_GPIO               14      //GPIO23

int duty_cycle = 64;

double get_current_value(int voltage, float duty_value, float pwm_duty_resolution)
{
    double duty_cycle = 0.0;
    double RMS_value = 0.0;

    duty_cycle = duty_value / pwm_duty_resolution;

    RMS_value = (1800 - voltage) / (sqrt(duty_cycle));

    return RMS_value;
}

void app_main() 
{
    PWM_init(PWM_DUTY_RES_BIT, PWM_PIN, PWM_FREQUENCY);

    ADC1_init(ADC_UNIT, ADC_BITWIDTH, ADC_ATTEN);
    set_adc_pin(ADC_CURRENT_PIN, ADC_VOLTAGE_PIN);

    Button_Init(BUTTON_0_GPIO, BUTTON_1_GPIO);
   
    while(1)
    {
        if(eButton_Read(BUTTON_0) == PRESSED)
        {
            duty_cycle++;
        }
        else if(eButton_Read(BUTTON_1) == PRESSED)
        {
            duty_cycle--;
        }
        else
        {
            duty_cycle = duty_cycle;
        }
        
        PWM_duty_cycle(duty_cycle);

        vTaskDelay(50);

        printf("duty_cycle = %d \n",duty_cycle);
        //printf("ADC value = %d \n", adc_read_voltage(ADC_CURRENT_PIN, ADC_SAMPLES_NUMBER));
        printf("RMS value = %f \n", get_current_value(adc_read_voltage(ADC_CURRENT_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES));

    }
}