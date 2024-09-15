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

bool measure_flag = false;

void measure_status(void *arg)
{
    if(gpio_get_level(GPIO_NUM_22) == true)
    {
        measure_flag = true;
    }
    else
    {
        measure_flag = false;
    }
}

float get_current_value(void)
{
    uint16_t sample_counter = 0;
    uint32_t buffer = 0;
    float result = 0.0;

    gpio_set_direction(GPIO_NUM_22, GPIO_MODE_INPUT);
    gpio_install_isr_service(0);
    gpio_set_intr_type(GPIO_NUM_22, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(GPIO_NUM_22, measure_status, NULL);

    while(1)
    {
        if(measure_flag == true && sample_counter < 1000)
        {
            buffer = buffer + adc_read_voltage(ADC_INPUT_VOLTAGE_PIN);
            sample_counter++;

            esp_rom_delay_us(10);
        }
        else if(sample_counter >= 1000)
        {
            gpio_intr_disable(GPIO_NUM_22);
            gpio_uninstall_isr_service();
            gpio_reset_pin(GPIO_NUM_22);

            result = buffer/1000;
            result = (result - 2500)*-1.0;
            result = result * 0.707; 

            return result;
        }
        else
        {
            buffer = buffer;
            sample_counter = sample_counter;
        }
    }  
}
 
void app_main() 
{
    PWM_init(PWM_DUTY_RES, PWM_PIN, PWM_FREQUENCY);

    ADC1_init(ADC_UNIT, ADC_BITWIDTH, ADC_ATTEN);
    set_adc_pin(ADC_INPUT_VOLTAGE_PIN, ADC_OUTPUT_VOLTAGE_PIN);
   
  
    
    while(1)
    {
        PWM_duty_cycle(512);

        printf("RMS value = %f \n", get_current_value());

        vTaskDelay(100);
    }
}