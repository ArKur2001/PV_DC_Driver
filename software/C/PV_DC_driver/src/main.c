#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include "PWM/pwm.h"
#include <ADC/adc.h>
#include "BUTTONS/buttons.h"
#include <MEASUREMENTS/measurements.h>

#include "LCD/HD44780.h"
#include <stdio.h>
#include <string.h>

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

#define BUTTON_0_GPIO               27      //GPIO23     
#define BUTTON_1_GPIO               14      //GPIO23

#define VOLTAGE_MULTIPLIER          20      //voltage_divider_value
#define VOLTAGE_REF_LVL             900     //voltage_potentiometer_ref(mV)
#define CURRENT_REF_LVL             900     //current_potentiometer_ref(mV)

#define LCD_ADDR                    0x27
#define SDA_PIN                     21
#define SCL_PIN                     22
#define LCD_COLS                    20
#define LCD_ROWS                    4

int duty_cycle = 64;
double voltage_value = 0.0;
double current_value = 0.0;

void build_string(char *buffer, int width, double number, const char *unit, int precision)
{
    char format[10];
    snprintf(format, sizeof(format), "%%.%df%%s", precision);

    char temporary[20];
    snprintf(temporary, sizeof(temporary), format, number, unit);

    snprintf(buffer, width, "%-*s", width, temporary); 
}

void LCD_BOILER_CAPACITY_state()
{
    LCD_clearScreen();
    LCD_setCursor(0, 0);
    LCD_writeStr("  Boiler capacity:  ");
}

void LCD_BOILER_CAPACITY_setting(uint16_t capacity)
{
    char str_buf[5];

    build_string(str_buf, 7, capacity, " L", 0);
    LCD_setCursor(8, 2);
    LCD_writeStr(str_buf);
}

void LCD_TEMPERATURE_state()
{
    LCD_clearScreen();
    LCD_setCursor(0, 0);
    LCD_writeStr("Temperature setting:");
}

void LCD_TEMPERATURE_setting(uint16_t temp)
{
    char str_buf[6];
    char symbol[3];
    symbol[0] = ' ';
    symbol[1] = 223;
    symbol[2] = 'C';
    
    build_string(str_buf, 8, temp, symbol, 0);
    LCD_setCursor(8, 2);
    LCD_writeStr(str_buf);
}

void LCD_INFO_state()
{
    LCD_clearScreen();
    LCD_setCursor(0, 0);
    LCD_writeStr("P:        T:        ");
    LCD_setCursor(0, 1);
    LCD_writeStr("U:        I:        ");
    LCD_setCursor(0, 2);
    LCD_writeStr("Etot:               ");
    LCD_setCursor(0, 3);
    LCD_writeStr("Heating end:        ");
}

void LCD_INFO_power(double power)
{
    char str_buf[7];

    build_string(str_buf, 7, power, " W", 0);
    LCD_setCursor(3, 0);
    LCD_writeStr(str_buf);
}

void LCD_INFO_temp(double temp)
{
    char str_buf[7];
    char symbol[3];
    symbol[0] = ' ';
    symbol[1] = 223;
    symbol[2] = 'C';
    
    build_string(str_buf, 8, temp, symbol, 1);
    LCD_setCursor(13, 0);
    LCD_writeStr(str_buf);
}

void LCD_INFO_voltage(double voltage)
{
    char str_buf[7];
    
    build_string(str_buf, 6, voltage, " V", 0);
    LCD_setCursor(3, 1);
    LCD_writeStr(str_buf);
}

void LCD_INFO_current(double current)
{
    char str_buf[7];
    
    build_string(str_buf, 7, current, " A", 1);
    LCD_setCursor(13, 1);
    LCD_writeStr(str_buf);
}

void LCD_INFO_energy(double energy_j)
{
    char str_buf[14];
    double energy_kwh = energy_j / 3600000.0;

    build_string(str_buf, 14, energy_kwh, " kWh", 1);
    LCD_setCursor(6, 2);
    LCD_writeStr(str_buf);
}

void LCD_INFO_hours(uint8_t hour)
{
    char str_buf[3];
    
    build_string(str_buf, 3, hour, "h", 0);
    LCD_setCursor(13, 3);
    LCD_writeStr(str_buf);
}

void LCD_INFO_minutes(uint8_t minute)
{
    char str_buf[3];

    build_string(str_buf, 3, minute, "m", 0);
    LCD_setCursor(17, 3);
    LCD_writeStr(str_buf);
}

void app_main() 
{
    PWM_init(PWM_DUTY_RES_BIT, PWM_PIN, PWM_FREQUENCY);

    ADC1_init(ADC_UNIT, ADC_BITWIDTH, ADC_ATTEN);
    set_adc_pin(ADC_CURRENT_PIN, ADC_VOLTAGE_PIN);

    Button_Init(BUTTON_0_GPIO, BUTTON_1_GPIO);

    measurements_init(VOLTAGE_REF_LVL, VOLTAGE_MULTIPLIER, CURRENT_REF_LVL);

    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
   
    //LCD_INFO_state();
    //LCD_TEMPERATURE_state();
    LCD_BOILER_CAPACITY_state();

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

        vTaskDelay(25);

        voltage_value = get_voltage_value(adc_read_voltage(ADC_VOLTAGE_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES);
        current_value = get_current_value(adc_read_voltage(ADC_CURRENT_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES);

        printf("duty_cycle = %d \n",duty_cycle);
        printf("Voltage RMS value = %f V\n", voltage_value);
        printf("Current RMS value = %f A\n", current_value);

        //LCD_INFO_power(1678.66666);
        //LCD_INFO_temp(85.7234);
        //LCD_INFO_voltage(voltage_value);
        //LCD_INFO_current(current_value);
        //LCD_INFO_energy(9500000);
        //LCD_INFO_hours(2);
        //LCD_INFO_minutes(2);

        //LCD_TEMPERATURE_setting(85);
        LCD_BOILER_CAPACITY_setting(100);
    }
}