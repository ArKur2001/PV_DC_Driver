#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include "PWM/pwm.h"
#include <ADC/adc.h>
#include "BUTTONS/buttons.h"
#include <MEASUREMENTS/measurements.h>
#include "LCD/HD44780.h"
#include "LCD/LCD_string.h"
#include "DS18B20/ds18b20.h"

#include <stdio.h>
#include "driver/gpio.h" //gpio

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

#define BUTTON_0_GPIO               27      //GPIO27     
#define BUTTON_1_GPIO               14      //GPIO14

#define VOLTAGE_MULTIPLIER          20      //voltage_divider_value
#define VOLTAGE_REF_LVL             900     //voltage_potentiometer_ref(mV)
#define CURRENT_REF_LVL             900     //current_potentiometer_ref(mV)

#define LCD_ADDR                    0x27
#define SDA_PIN                     21
#define SCL_PIN                     22
#define LCD_COLS                    20
#define LCD_ROWS                    4

#define DS18B20_GPIO1               4       //boiler temperature sensor
#define DS18B20_GPIO2               5       //case temperature sensor

int duty_cycle = 64;
double voltage_value = 0.0;
double current_value = 0.0;

void app_main() 
{
    PWM_init(PWM_DUTY_RES_BIT, PWM_PIN, PWM_FREQUENCY);

    ADC1_init(ADC_UNIT, ADC_BITWIDTH, ADC_ATTEN);
    set_adc_pin(ADC_CURRENT_PIN, ADC_VOLTAGE_PIN);

    Button_Init(BUTTON_0_GPIO, BUTTON_1_GPIO);

    measurements_init(VOLTAGE_REF_LVL, VOLTAGE_MULTIPLIER, CURRENT_REF_LVL);

    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
   
    LCD_INFO_state();
    //LCD_TEMPERATURE_state();
    //LCD_BOILER_CAPACITY_state();

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

        vTaskDelay(pdMS_TO_TICKS(25));

        voltage_value = get_voltage_value(adc_read_voltage(ADC_VOLTAGE_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES);
        current_value = get_current_value(adc_read_voltage(ADC_CURRENT_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES);

        printf("duty_cycle = %d \n",duty_cycle);
        printf("Voltage RMS value = %f V\n", voltage_value);
        printf("Current RMS value = %f A\n", current_value);

        float temperature1 = ds18b20_get_temp(DS18B20_GPIO1);
        float temperature2 = ds18b20_get_temp(DS18B20_GPIO2);

        printf("Temp_boiler = %f C\n", temperature1);
        printf("Temp_case = %f C\n", temperature2);

        LCD_INFO_power(1678.66666);
        LCD_INFO_temp(temperature1);
        LCD_INFO_voltage(voltage_value);
        LCD_INFO_current(current_value);
        LCD_INFO_energy(9500000);
        LCD_INFO_hours(2);
        LCD_INFO_minutes(2);

        //LCD_TEMPERATURE_setting(20.345);

        //LCD_BOILER_CAPACITY_setting(100);
    }
}