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

#define BUTTON_0_GPIO               26      //GPIO26     
#define BUTTON_1_GPIO               27      //GPIO27
#define BUTTON_2_GPIO               14      //GPIO14

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

enum Program_state  {IDLE, READ_TEMP_BOILER, READ_TEMP_CASE, MEASUREMENTS, MPPT, READ_BUTTONS ,UPDATE_SCREEN};
enum LCD_state      {INFO, TEMPERATURE, BOILER_CAPACITY};

void app_main() 
{
    enum Program_state  eProgram_state  = IDLE;
    enum LCD_state      eLCD_state      = INFO;  

    uint64_t loop_number = 0;

    int duty_cycle = 64;

    double voltage_value = 0.0;
    double current_value = 0.0;
    double power_value = 0.0;

    float temp_water = 0.0;
    float temp_case = 0.0; 

    uint8_t desired_temperature = 50;
    uint8_t boiler_capacity = 100;

    PWM_init(PWM_DUTY_RES_BIT, PWM_PIN, PWM_FREQUENCY);

    ADC1_init(ADC_UNIT, ADC_BITWIDTH, ADC_ATTEN);
    set_adc_pin(ADC_CURRENT_PIN, ADC_VOLTAGE_PIN);

    Button_Init(BUTTON_0_GPIO, BUTTON_1_GPIO, BUTTON_2_GPIO);

    measurements_init(VOLTAGE_REF_LVL, VOLTAGE_MULTIPLIER, CURRENT_REF_LVL);

    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
   
    LCD_INFO_state();
    //LCD_TEMPERATURE_state();
    //LCD_BOILER_CAPACITY_state();

    while(1)
    {
        switch (eProgram_state)
        {
            case IDLE:
                if(loop_number % 101 == 0)
                {
                    eProgram_state = READ_TEMP_BOILER;
                }
                else if(loop_number % 211 == 0)
                {
                    eProgram_state = READ_TEMP_CASE;
                }
                else if(loop_number % 11 == 0)
                {
                    eProgram_state = READ_BUTTONS;
                }
                else
                {
                    eProgram_state = MEASUREMENTS;
                }

                printf("loop_number = %" PRIu64 "\n",loop_number);

                loop_number++;

                break;

            case READ_TEMP_BOILER:
                temp_water = ds18b20_get_temp(DS18B20_GPIO1);

                printf("Temp_boiler = %f C\n", temp_water); 

                eProgram_state = MEASUREMENTS;

                break;

            case READ_TEMP_CASE:
                temp_case = ds18b20_get_temp(DS18B20_GPIO2);

                printf("Temp_case = %f C\n", temp_case); 

                eProgram_state = MEASUREMENTS;

                break;

            case MEASUREMENTS:
                vTaskDelay(pdMS_TO_TICKS(25));

                voltage_value = get_voltage_value(adc_read_voltage(ADC_VOLTAGE_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES);
                current_value = get_current_value(adc_read_voltage(ADC_CURRENT_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES);
                power_value = voltage_value * current_value;
                //printf("Voltage RMS value = %f V\n", voltage_value);
                //printf("Current RMS value = %f A\n", current_value);    

                eProgram_state = MPPT;
                break;

            case MPPT:
                PWM_duty_cycle(duty_cycle);

                //printf("duty_cycle = %d \n",duty_cycle);

                eProgram_state = IDLE;
                break;

            case READ_BUTTONS:
                if(eButton_Read(BUTTON_0) == PRESSED)
                {
                    switch (eLCD_state)
                    {
                        case INFO:
                                desired_temperature = desired_temperature;
                                boiler_capacity = boiler_capacity;

                            break;

                        case TEMPERATURE:
                                if(desired_temperature <= 0)
                                {
                                    desired_temperature = desired_temperature;
                                }
                                else
                                {
                                    desired_temperature--;
                                }

                            break;

                         case BOILER_CAPACITY:
                                if(boiler_capacity <= 0)
                                {
                                    boiler_capacity = boiler_capacity;
                                }
                                else
                                {
                                    boiler_capacity = boiler_capacity - 10;
                                }

                            break;

                        default:
                            desired_temperature = desired_temperature;
                            boiler_capacity = boiler_capacity;

                            break;
                    }
                }
                else if(eButton_Read(BUTTON_2) == PRESSED)
                {
                    switch (eLCD_state)
                    {
                        case INFO:
                                desired_temperature = desired_temperature;
                                boiler_capacity = boiler_capacity;

                            break;

                        case TEMPERATURE:
                                if(desired_temperature >= 85)
                                {
                                    desired_temperature = desired_temperature;
                                }
                                else
                                {
                                    desired_temperature++;
                                }

                            break;

                         case BOILER_CAPACITY:
                                
                                boiler_capacity = boiler_capacity + 10;
                        
                            break;

                        default:
                            desired_temperature = desired_temperature;
                            boiler_capacity = boiler_capacity;

                            break;
                    }
                }
                else
                {
                    desired_temperature = desired_temperature;
                    boiler_capacity = boiler_capacity;
                }

                eProgram_state = UPDATE_SCREEN;
                break;

            case UPDATE_SCREEN:
                if(eButton_Read(BUTTON_1) == PRESSED)
                {
                    switch (eLCD_state)
                    {
                        case INFO:
                                eLCD_state = TEMPERATURE;
                                LCD_TEMPERATURE_state();

                            break;

                        case TEMPERATURE:
                                eLCD_state = BOILER_CAPACITY;
                                LCD_BOILER_CAPACITY_state();

                            break;

                        case BOILER_CAPACITY:
                                eLCD_state = INFO;
                                LCD_INFO_state();

                            break;    

                        default:
                                eLCD_state = INFO;
                                LCD_INFO_state();

                            break;
                    }
                }
                else
                {
                    eLCD_state = eLCD_state;
                }

                switch (eLCD_state)
                {
                    case INFO:
                        LCD_INFO_power(power_value);
                        LCD_INFO_temp(temp_water);
                        LCD_INFO_voltage(voltage_value);
                        LCD_INFO_current(current_value);
                        LCD_INFO_energy(9500000);
                        LCD_INFO_hours(2);
                        LCD_INFO_minutes(2);

                        break;

                    case TEMPERATURE:
                        LCD_TEMPERATURE_setting(desired_temperature);

                        break;

                    case BOILER_CAPACITY:
                        LCD_BOILER_CAPACITY_setting(boiler_capacity);

                        break;
                    
                    default:
                        eLCD_state = INFO;

                        break;
                }

                eProgram_state = IDLE;

                break;
            
            default:
                eProgram_state = IDLE;

                break;
        }
    }
}