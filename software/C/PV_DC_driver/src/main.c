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

#include <stdio.h>
#include "driver/gpio.h" //gpio

#include "rom/ets_sys.h"

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

int ds18b20_reset(gpio_num_t pin) 
{
    int response = 0;
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    ets_delay_us(480);
    gpio_set_level(pin, 1);
    ets_delay_us(70);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    response = gpio_get_level(pin);
    ets_delay_us(410);
    
    if(response == 0)
    {
        return 1;
    }
    else
    {
        return -1;
    } 
}

void ds18b20_write_bit(gpio_num_t pin, int bit) 
{
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);

    if(bit == true)
    {
        ets_delay_us(10);
    }
    else
    {
        ets_delay_us(65);
    }

    gpio_set_level(pin, 1);

    if(bit == true)
    {
        ets_delay_us(55);
    }
    else
    {
        ets_delay_us(5);
    }
}

int ds18b20_read_bit(gpio_num_t pin) {
    int bit = 0;
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    ets_delay_us(3);
    gpio_set_level(pin, 1);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    ets_delay_us(10);
    bit = gpio_get_level(pin);
    ets_delay_us(45);

    return bit;
}

void ds18b20_write_byte(gpio_num_t pin, int byte) {
    for (int i = 0; i < 8; i++) 
    {
        ds18b20_write_bit(pin, byte & 0x01);
        byte >>= 1;
    }
}

int ds18b20_read_byte(gpio_num_t pin) {
    int byte = 0;
    for (int i = 0; i < 8; i++) 
    {
        byte |= (ds18b20_read_bit(pin) << i);
    }

    return byte;
}

float ds18b20_get_temp(gpio_num_t pin) {
    if (ds18b20_reset(pin) == 1) {
        ds18b20_write_byte(pin, 0xCC); // SKIP ROM
        ds18b20_write_byte(pin, 0x44); // CONVERT T
        vTaskDelay(pdMS_TO_TICKS(375)); // Czas konwersji dla 11 bitów

        ds18b20_reset(pin);
        ds18b20_write_byte(pin, 0xCC); // SKIP ROM
        ds18b20_write_byte(pin, 0xBE); // READ SCRATCHPAD

        int temp_LSB = ds18b20_read_byte(pin);
        int temp_MSB = ds18b20_read_byte(pin);
        int16_t temp = (temp_MSB << 8) | temp_LSB;

        return temp / 16.0; // Celcius degree conversion
    } 
    else 
    {
        return -10.0; // Error code
    }
}

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