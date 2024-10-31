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
#include "MEMORY/memory.h"
#include "LED/led.h"
#include "esp_timer.h"

#include <stdio.h>

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
#define MEASUREMENT_DELAY           50      //3 time constants (ms)

#define BUTTON_0_GPIO               26      //GPIO26     
#define BUTTON_1_GPIO               27      //GPIO27
#define BUTTON_2_GPIO               14      //GPIO14

#define VOLTAGE_MULTIPLIER          20      //voltage_divider_value
#define VOLTAGE_REF_LVL             900     //voltage_potentiometer_ref(mV)
#define CURRENT_REF_LVL             900     //current_potentiometer_ref(mV)

#define LCD_ADDR                    0x27
#define SDA_PIN                     21      //GPIO21
#define SCL_PIN                     22      //GPIO22
#define LCD_COLS                    20
#define LCD_ROWS                    4

#define DS18B20_GPIO1               4       //boiler temperature sensor
#define DS18B20_GPIO2               5       //case temperature sensor

#define SPECIFIC_HEAT_WATER         4200    //J/kg
#define WRITE_PERIOD                3600    //s
#define LCD_BACKLIGHT_PERIOD        60      //s

#define LED_GREEN_PIN               18      //GPIO18
#define LED_RED_PIN                 19      //GPIO19

#define MPPT_PERIOD                 15      //s

enum Program_state              {IDLE, READ_TEMP_BOILER, END_TIME, READ_TEMP_CASE, MEASUREMENTS, MPPT, READ_BUTTONS, UPDATE_SCREEN, WRITE_MEMORY, HEATING_STATUS, CALLIB};
enum LCD_state                  {INFO, TEMPERATURE, BOILER_CAPACITY};
enum Water_Heating_Status       {STOP_HEATING, ALLOW_HEATING};
enum MPPT_stage                 {STAGE_SETUP, STAGE_1, STAGE_2, STAGE_3, STAGE_4, STAGE_5, STAGE_6, STAGE_7, STAGE_8, STAGE_9, STAGE_10, STAGE_11, STAGE_12};

enum Program_state  eProgram_state              = IDLE;
enum LCD_state      eLCD_state                  = INFO;  
enum Backligt_state eBacklight_state            = ON;
enum Water_Heating_Status eWater_Heating_Status = STOP_HEATING;
enum MPPT_stage eMPPT_stage                     = STAGE_SETUP;

uint64_t loop_number = 0;
uint64_t second_number = 0;
uint64_t second_number_lcd_tmp = 0;
uint64_t second_number_mppt_tmp = 0;

double voltage_value = 0.0;
double current_value = 0.0;
double power_value = 0.0;

float temp_water = 0.0;
float previous_temp_water = 0.0;
float temp_case = 0.0; 

uint8_t desired_temperature = 50;
uint16_t boiler_capacity = 100;

uint8_t hours = 0;
uint8_t minutes = 0;

uint64_t energy_j = 0.0;

uint8_t duty_cycle = 0;
uint8_t duty_cycle_tmp1 = 0;
uint8_t duty_cycle_tmp2 = 0;
uint8_t duty_cycle_opt = 0;
double power_buf = 0.0;
double power_opt = 0.0;

void timer_callback(void *param)
{
    second_number++;
    //printf("second_number = %" PRIu64 "\n", second_number);

    energy_j = energy_j + power_value;

    printf("energy = %" PRIu64 " J\n", energy_j);
}

void app_main() 
{
    PWM_init(PWM_DUTY_RES_BIT, PWM_PIN, PWM_FREQUENCY);
    PWM_duty_cycle(0);

    ADC1_init(ADC_UNIT, ADC_BITWIDTH, ADC_ATTEN);
    set_adc_pin(ADC_CURRENT_PIN, ADC_VOLTAGE_PIN);

    Button_Init(BUTTON_0_GPIO, BUTTON_1_GPIO, BUTTON_2_GPIO);

    measurements_init(VOLTAGE_REF_LVL, VOLTAGE_MULTIPLIER, CURRENT_REF_LVL);

    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    
    Led_Init(LED_GREEN_PIN, LED_RED_PIN);

    flash_read(&desired_temperature, &boiler_capacity, &energy_j);

    const esp_timer_create_args_t program_timer_args = {
      .callback = &timer_callback,
      .name = "Program_timer"};

    esp_timer_handle_t program_timer_handler;
    esp_timer_create(&program_timer_args, &program_timer_handler);
    esp_timer_start_periodic(program_timer_handler, 1000000);

    LCD_INFO_state();

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
                else if(second_number % WRITE_PERIOD == 0)
                {
                    eProgram_state = WRITE_MEMORY;
                }
                else
                {
                    eProgram_state = MEASUREMENTS;
                }

                printf("loop_number = %" PRIu64 "\n", loop_number);

                loop_number++;

                break;

            case READ_TEMP_BOILER:
                temp_water = ds18b20_get_temp(DS18B20_GPIO1);

                printf("Temp_boiler = %f C\n", temp_water); 

                eProgram_state = HEATING_STATUS;

                break;

            case END_TIME:
                if(power_value == 0.0)
                {
                    hours = 0;
                    minutes = 0;
                }
                else if(previous_temp_water != temp_water)
                {
                    int32_t time = ((desired_temperature - temp_water) * boiler_capacity * SPECIFIC_HEAT_WATER) / power_value;

                    hours = time / 3600;
                    minutes = (time - (hours * 3600)) / 60;

                    previous_temp_water = temp_water;
                }
                else
                {
                    previous_temp_water = temp_water;
                }
    
                eProgram_state = UPDATE_SCREEN;

                break;

            case READ_TEMP_CASE:
                temp_case = ds18b20_get_temp(DS18B20_GPIO2);

                printf("Temp_case = %f C\n", temp_case); 

                eProgram_state = HEATING_STATUS;

                break;

            case MEASUREMENTS:
                vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_DELAY));

                if(duty_cycle != 0)
                {
                    voltage_value = get_voltage_value(adc_read_voltage(ADC_VOLTAGE_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES);
                    current_value = get_current_value(adc_read_voltage(ADC_CURRENT_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES);
                    power_value = voltage_value * current_value;
                }
                else
                {
                    voltage_value = get_voltage_value(adc_read_voltage(ADC_VOLTAGE_PIN, ADC_SAMPLES_NUMBER), 128, PWM_DUTY_RES);
                    current_value = 0.0;
                    power_value = 0.0;
                }
               
                //printf("Voltage RMS value = %f V\n", voltage_value);
                //printf("Current RMS value = %f A\n", current_value); 
                //printf("Power RMS value = %f W\n", power_value);    

                if(eMPPT_stage == STAGE_SETUP)
                {
                    eProgram_state = IDLE;
                }
                else
                {
                    eProgram_state = MPPT;
                }

                break;

            case MPPT:
                uint8_t duty_cycle_temporary = 0;
                double power_temporary = 0.0;

                switch (eWater_Heating_Status)
                {
                    case STOP_HEATING:
                        duty_cycle = 0;
                        
                        eProgram_state = IDLE;

                        break;

                     case ALLOW_HEATING:
                        switch (eMPPT_stage)
                        {
                            case STAGE_SETUP:
                                if(second_number_mppt_tmp > second_number)
                                {
                                    duty_cycle = duty_cycle;

                                    eMPPT_stage = STAGE_SETUP;

                                    eProgram_state = IDLE;
                                }
                                else
                                {
                                    duty_cycle_tmp1 = 32;
                                    duty_cycle_tmp2 = 96;
                                    duty_cycle_opt = 0;

                                    power_buf = 0.0;
                                    power_opt = 0.0;
                                
                                    duty_cycle = duty_cycle_tmp1;

                                    printf("Power RMS value = %f W\n", power_value);

                                    eMPPT_stage = STAGE_1;
                                    eProgram_state = MEASUREMENTS;
                                }

                                break;

                            case STAGE_1:
                                power_buf = power_value;

                                duty_cycle = duty_cycle_tmp2;

                                printf("Power RMS value = %f W\n", power_value);

                                eMPPT_stage = STAGE_2;
                                eProgram_state = MEASUREMENTS;

                                break;

                            case STAGE_2:
                                if(power_value > power_buf)
                                {
                                    duty_cycle_temporary = duty_cycle_tmp2;
                                    power_temporary = power_value;
                                }
                                else
                                {
                                    duty_cycle_temporary = duty_cycle_tmp1;
                                    power_temporary = power_buf;
                                }

                                printf("Power RMS value = %f W\n", power_value);

                                power_opt = power_temporary;
                                duty_cycle_opt = duty_cycle_temporary;

                                duty_cycle_tmp1 = duty_cycle_temporary - 16;
                                duty_cycle_tmp2 = duty_cycle_temporary + 16;

                                duty_cycle = duty_cycle_tmp1;

                                eMPPT_stage = STAGE_3;
                                eProgram_state = MEASUREMENTS;

                                break;

                            case STAGE_3:
                                power_buf = power_value;

                                duty_cycle = duty_cycle_tmp2;

                                printf("Power RMS value = %f W\n", power_value);

                                eMPPT_stage = STAGE_4;
                                    
                                eProgram_state = MEASUREMENTS;

                                break;

                            case STAGE_4:
                                if(power_value > power_buf)
                                {
                                    duty_cycle_temporary = duty_cycle_tmp2;
                                    power_temporary = power_value;
                                }
                                else
                                {
                                    duty_cycle_temporary = duty_cycle_tmp1;
                                    power_temporary = power_buf;
                                }


                                if(power_temporary > power_opt)
                                {
                                    power_opt = power_temporary;
                                    duty_cycle_opt = duty_cycle_temporary;
                                }
                                else
                                {
                                    power_opt = power_opt;
                                    duty_cycle_opt = duty_cycle_opt;
                                }

                                printf("Power RMS value = %f W\n", power_value);

                                duty_cycle_tmp1 = duty_cycle_temporary - 8;
                                duty_cycle_tmp2 = duty_cycle_temporary + 8;

                                duty_cycle = duty_cycle_tmp1;

                                eMPPT_stage = STAGE_5;   
                                eProgram_state = MEASUREMENTS;

                                break;

                            case STAGE_5:
                                power_buf = power_value;

                                duty_cycle = duty_cycle_tmp2;

                                printf("Power RMS value = %f W\n", power_value);

                                eMPPT_stage = STAGE_6;
                                eProgram_state = MEASUREMENTS;

                                break;

                            case STAGE_6:
                                if(power_value > power_buf)
                                {
                                    duty_cycle_temporary = duty_cycle_tmp2;
                                    power_temporary = power_value;
                                }
                                else
                                {
                                    duty_cycle_temporary = duty_cycle_tmp1;
                                    power_temporary = power_buf;
                                }


                                if(power_temporary > power_opt)
                                {
                                    power_opt = power_temporary;
                                    duty_cycle_opt = duty_cycle_temporary;
                                }
                                else
                                {
                                    power_opt = power_opt;
                                    duty_cycle_opt = duty_cycle_opt;
                                }

                                printf("Power RMS value = %f W\n", power_value);

                                duty_cycle_tmp1 = duty_cycle_temporary - 4;
                                duty_cycle_tmp2 = duty_cycle_temporary + 4;

                                duty_cycle = duty_cycle_tmp1;

                                eMPPT_stage = STAGE_7;
                                eProgram_state = MEASUREMENTS;

                                break;

                            case STAGE_7:
                                power_buf = power_value;

                                duty_cycle = duty_cycle_tmp2;

                                printf("Power RMS value = %f W\n", power_value);

                                eMPPT_stage = STAGE_8;
                                eProgram_state = MEASUREMENTS;

                                break;

                            case STAGE_8:
                                if(power_value > power_buf)
                                {
                                    duty_cycle_temporary = duty_cycle_tmp2;
                                    power_temporary = power_value;
                                }
                                else
                                {
                                    duty_cycle_temporary = duty_cycle_tmp1;
                                    power_temporary = power_buf;
                                }


                                if(power_temporary > power_opt)
                                {
                                    power_opt = power_temporary;
                                    duty_cycle_opt = duty_cycle_temporary;
                                }
                                else
                                {
                                    power_opt = power_opt;
                                    duty_cycle_opt = duty_cycle_opt;
                                }

                                printf("Power RMS value = %f W\n", power_value);

                                duty_cycle_tmp1 = duty_cycle_temporary - 2;
                                duty_cycle_tmp2 = duty_cycle_temporary + 2;

                                duty_cycle = duty_cycle_tmp1;

                                eMPPT_stage = STAGE_9;
                                eProgram_state = MEASUREMENTS;

                                break;

                            case STAGE_9:
                                power_buf = power_value;

                                duty_cycle = duty_cycle_tmp2;

                                printf("Power RMS value = %f W\n", power_value);

                                eMPPT_stage = STAGE_10;
                                eProgram_state = MEASUREMENTS;

                                break;

                            case STAGE_10:
                                if(power_value > power_buf)
                                {
                                    duty_cycle_temporary = duty_cycle_tmp2;
                                    power_temporary = power_value;
                                }
                                else
                                {
                                    duty_cycle_temporary = duty_cycle_tmp1;
                                    power_temporary = power_buf;
                                }


                                if(power_temporary > power_opt)
                                {
                                    power_opt = power_temporary;
                                    duty_cycle_opt = duty_cycle_temporary;
                                }
                                else
                                {
                                    power_opt = power_opt;
                                    duty_cycle_opt = duty_cycle_opt;
                                }

                                printf("Power RMS value = %f W\n", power_value);

                                duty_cycle_tmp1 = duty_cycle_temporary - 1;
                                duty_cycle_tmp2 = duty_cycle_temporary + 1;

                                duty_cycle = duty_cycle_tmp1;


                                eMPPT_stage = STAGE_11;
                                eProgram_state = MEASUREMENTS;

                                break;
                                
                            case STAGE_11:
                                power_buf = power_value;

                                duty_cycle = duty_cycle_tmp2;

                                printf("Power RMS value = %f W\n", power_value);

                                eMPPT_stage = STAGE_12;
                                eProgram_state = MEASUREMENTS;

                                break;

                            case STAGE_12:
                                if(power_value > power_buf)
                                {
                                    duty_cycle_temporary = duty_cycle_tmp2;
                                    power_temporary = power_value;
                                }
                                else
                                {
                                    duty_cycle_temporary = duty_cycle_tmp1;
                                    power_temporary = power_buf;
                                }


                                if(power_temporary > power_opt)
                                {
                                    power_opt = power_temporary;
                                    duty_cycle_opt = duty_cycle_temporary;
                                }
                                else
                                {
                                    power_opt = power_opt;
                                    duty_cycle_opt = duty_cycle_opt;
                                }

                                printf("Power RMS value = %f W\n", power_value);

                                duty_cycle_tmp1 = 0;
                                duty_cycle_tmp2 = 0;
                                power_opt = 0.0;

                                duty_cycle = duty_cycle_opt;

                                second_number_mppt_tmp = second_number_mppt_tmp + MPPT_PERIOD;

                                eMPPT_stage = STAGE_SETUP;
                                eProgram_state = MEASUREMENTS;

                                break;

                            default:
                                duty_cycle = 0;

                                second_number_mppt_tmp = second_number_mppt_tmp;

                                eMPPT_stage = STAGE_SETUP;
                                eProgram_state = IDLE;

                                break;
                        }

                        break;

                    default:
                        duty_cycle = 0;

                        break;
                }

                PWM_duty_cycle(duty_cycle);

                printf("duty_cycle = %" PRIu8 "\n",duty_cycle);
                
                break;

            case READ_BUTTONS:
                if(eButton_Read(BUTTON_0) == PRESSED)
                {
                    switch (eLCD_state)
                    {
                        case INFO:
                                desired_temperature = desired_temperature;
                                boiler_capacity = boiler_capacity;
                                
                                eBacklight_state = ON;

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

                                eBacklight_state = ON;

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

                                eBacklight_state = ON;

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

                                eBacklight_state = ON;

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

                                eBacklight_state = ON;

                            break;

                         case BOILER_CAPACITY:
                                
                                boiler_capacity = boiler_capacity + 10;

                                eBacklight_state = ON;
                        
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

                eProgram_state = END_TIME;
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

                    eBacklight_state = ON;
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
                        LCD_INFO_energy(energy_j);
                        LCD_INFO_time(hours, minutes);

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

                switch (eBacklight_state)
                {
                    case ON:
                        second_number_lcd_tmp = second_number + LCD_BACKLIGHT_PERIOD;
                        
                        eBacklight_state = OFF;

                        if(second_number_lcd_tmp > second_number)
                        {
                            set_backlight_state(ON);
                        }
                        else
                        {
                            set_backlight_state(OFF);
                        }

                        break;

                    case OFF:
                        if(second_number_lcd_tmp > second_number)
                        {
                            set_backlight_state(ON);
                        }
                        else
                        {
                            set_backlight_state(OFF);
                        }

                        break;
                    
                    default:
                        second_number_lcd_tmp = second_number_lcd_tmp;

                        break;
                }
                

                eProgram_state = IDLE;

                break;
            
            case WRITE_MEMORY:
                uint8_t desired_temperature_tmp;
                uint16_t boiler_capacity_tmp;
                uint64_t energy_j_tmp;

                flash_read(&desired_temperature_tmp, &boiler_capacity_tmp, &energy_j_tmp);

                if(desired_temperature_tmp != desired_temperature || boiler_capacity_tmp != boiler_capacity || energy_j_tmp != energy_j)
                {
                    flash_write(desired_temperature, boiler_capacity, energy_j);

                    eProgram_state = IDLE;
                }
                else
                {
                    eProgram_state = IDLE;
                }

                break;

            case HEATING_STATUS:
                switch (eWater_Heating_Status)
                {
                    case STOP_HEATING:
                        if(temp_case >= 75)
                        {
                            eWater_Heating_Status = STOP_HEATING;

                            Led_green_set(LED_OFF);
                            Led_red_set(LED_ON);
                        }
                        else if(temp_water >= desired_temperature - 1)
                        {
                            eWater_Heating_Status = STOP_HEATING;
                            
                            Led_green_set(LED_ON);
                            Led_red_set(LED_ON);
                        }
                        else
                        {
                            eWater_Heating_Status = ALLOW_HEATING;

                            Led_green_set(LED_ON);
                            Led_red_set(LED_OFF);
                        }

                        eProgram_state = MPPT;

                        break;

                    case ALLOW_HEATING:
                        if(temp_case >= 85)
                        {
                            eWater_Heating_Status = STOP_HEATING;

                            Led_green_set(LED_OFF);
                            Led_red_set(LED_ON);
                        }
                        else if(temp_water >= desired_temperature)
                        {
                            eWater_Heating_Status = STOP_HEATING;

                            Led_green_set(LED_ON);
                            Led_red_set(LED_ON);
                        }
                        else
                        {
                            eWater_Heating_Status = ALLOW_HEATING;

                            Led_green_set(LED_ON);
                            Led_red_set(LED_OFF);
                        }

                        eProgram_state = MPPT;

                        break;

                    default:
                        eWater_Heating_Status = STOP_HEATING;

                        Led_green_set(LED_OFF);
                        Led_red_set(LED_OFF);

                        eProgram_state = IDLE;

                        break;
                }

                break;

                case CALLIB:
                    duty_cycle = 64;
                    PWM_duty_cycle(duty_cycle);

                    voltage_value = get_voltage_value(adc_read_voltage(ADC_VOLTAGE_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES);
                    current_value = get_current_value(adc_read_voltage(ADC_CURRENT_PIN, ADC_SAMPLES_NUMBER), duty_cycle, PWM_DUTY_RES);
                    power_value = voltage_value * current_value;

                    printf("Voltage RMS value = %f V\n", voltage_value);
                    printf("Current RMS value = %f A\n", current_value); 
                    printf("Power RMS value = %f W\n", power_value); 

                    vTaskDelay(pdMS_TO_TICKS(500));  

                    eProgram_state = CALLIB;

                break;

            default:
                eProgram_state = IDLE;

                break;
        }
    }
}