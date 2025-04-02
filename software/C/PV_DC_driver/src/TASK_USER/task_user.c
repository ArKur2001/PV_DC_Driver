#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "TASK_USER/task_user.h"
#include "BUTTONS/buttons.h"
#include "LCD/HD44780.h"
#include "LCD/LCD_string.h"
#include "data_structures.h"

#define SPECIFIC_HEAT_WATER         4200    //J/kg
#define LCD_BACKLIGHT_PERIOD        60      //s

enum Task_User_state    {RECEIVE, UPDATE_BOILER_SETTINGS, END_TIME, UPDATE_SCREEN, SEND};
enum LCD_state          {INFO, TEMPERATURE, BOILER_CAPACITY};

void Update_Boiler_Settings(enum LCD_state eLCD_state, BoilerSettings *BoilerSettings_data, uint64_t second_number, uint64_t *second_number_lcd_tmp)
{
    if(eButton_Read(BUTTON_0) == PRESSED)
    {
        switch (eLCD_state)
        {
            case INFO:

                break;

            case TEMPERATURE:
                    if(BoilerSettings_data->desired_temperature > 10)
                    {
                        BoilerSettings_data->desired_temperature--;
                    }

                break;

            case BOILER_CAPACITY:
                    if(BoilerSettings_data->boiler_capacity > 10)
                    {
                        BoilerSettings_data->boiler_capacity -= 10;
                    }

                break;

            default:

                break;
        }
        *second_number_lcd_tmp = second_number + LCD_BACKLIGHT_PERIOD;
    }
    else if(eButton_Read(BUTTON_2) == PRESSED)
    {
        switch (eLCD_state)
        {
            case INFO:

                break;

            case TEMPERATURE:
                    if(BoilerSettings_data->desired_temperature < 85)
                    {
                        BoilerSettings_data->desired_temperature++;
                    }

                break;

                case BOILER_CAPACITY:
                    if( BoilerSettings_data->boiler_capacity < 10000)
                    {
                        BoilerSettings_data->boiler_capacity += 10;
                    }

                break;

            default:

                break;
        }
        *second_number_lcd_tmp = second_number + LCD_BACKLIGHT_PERIOD;
    }
}

void End_Time(BoilerSettings BoilerSettings_data, ElectricalMeasurements ElectricalMeasurements_data, TemperatureReadings TemperatureReadings_data,  uint8_t *hours, uint8_t *minutes)
{
    static float previous_temp_water = 0.0;
    static float previous_desired_water = 0.0;

    if(ElectricalMeasurements_data.power_value == 0.0)
    {
        *hours = 0;
        *minutes = 0;
    }
    else if(previous_temp_water != TemperatureReadings_data.temp_water || previous_desired_water != BoilerSettings_data.desired_temperature)
    {
        int32_t time = ((BoilerSettings_data.desired_temperature - TemperatureReadings_data.temp_water) * BoilerSettings_data.boiler_capacity * SPECIFIC_HEAT_WATER) / ElectricalMeasurements_data.power_value;
        *hours = time / 3600;
        *minutes = (time - (*hours * 3600)) / 60;
        previous_temp_water = TemperatureReadings_data.temp_water;
        previous_desired_water = BoilerSettings_data.desired_temperature;
    }
    else
    {
        previous_temp_water = TemperatureReadings_data.temp_water;
        previous_desired_water = BoilerSettings_data.desired_temperature;
    }
}

void Update_Screen(enum LCD_state *eLCD_state, BoilerSettings BoilerSettings_data, ElectricalMeasurements ElectricalMeasurements_data, TemperatureReadings TemperatureReadings_data, uint64_t second_number, uint64_t *second_number_lcd_tmp, uint64_t energy_j, uint8_t hours, uint8_t minutes)
{
    if(eButton_Read(BUTTON_1) == PRESSED)
    {
        switch (*eLCD_state)
        {
            case INFO:
                    *eLCD_state = TEMPERATURE;
                    LCD_TEMPERATURE_state();

                break;

            case TEMPERATURE:
                    *eLCD_state = BOILER_CAPACITY;
                    LCD_BOILER_CAPACITY_state();

                break;

            case BOILER_CAPACITY:
                    *eLCD_state = INFO;
                    LCD_INFO_state();

                break;    

            default:
                    *eLCD_state = INFO;
                    LCD_INFO_state();

                break;
        }

        *second_number_lcd_tmp = second_number + LCD_BACKLIGHT_PERIOD;
    }

    switch (*eLCD_state)
    {
        case INFO:
            LCD_INFO_power(ElectricalMeasurements_data.power_value);
            LCD_INFO_voltage(ElectricalMeasurements_data.voltage_value);
            LCD_INFO_current(ElectricalMeasurements_data.current_value);
            LCD_INFO_temp(TemperatureReadings_data.temp_water);
            LCD_INFO_energy(energy_j);
            LCD_INFO_time(hours, minutes); 

            break;

        case TEMPERATURE:
            LCD_TEMPERATURE_setting(BoilerSettings_data.desired_temperature);

            break;

        case BOILER_CAPACITY:
            LCD_BOILER_CAPACITY_setting(BoilerSettings_data.boiler_capacity);

            break;
        
        default:
            eLCD_state = INFO;

            break;
    }

    if(*second_number_lcd_tmp > second_number)
    {
        set_backlight_state(ON);
    }
    else
    {
        set_backlight_state(OFF);
    }
}

void Task_User(void *pvParameters)
{
    TaskUserParameters *params = (TaskUserParameters *)pvParameters;

    QueueHandle_t BoilerSettings_queue = params->BoilerSettings_queue;
    QueueHandle_t ElectricalMeasurements_queue = params->ElectricalMeasurements_queue;
    QueueHandle_t TemperatureReadings_queue = params->TemperatureReadings_queue;
    QueueHandle_t TimerData_queue = params->TimerData_queue;

    static BoilerSettings BoilerSettings_data = {50, 100};
    static ElectricalMeasurements ElectricalMeasurements_data = {0.0, 0.0, 0.0};
    static TemperatureReadings TemperatureReadings_data = {0.0, 0.0};
    static TimerData TimerData_data = {0, 0};

    static enum Task_User_state eTask_User_state = RECEIVE;
    static enum LCD_state eLCD_state = INFO;
    
    static uint8_t hours_tmp = 0;
    static uint8_t minutes_tmp = 0;

    static uint64_t second_number_lcd_tmp = LCD_BACKLIGHT_PERIOD;

    while(1)
    {
        switch (eTask_User_state)
        {
            case RECEIVE:
                if (xQueuePeek(BoilerSettings_queue, &BoilerSettings_data, pdMS_TO_TICKS(0)) == pdTRUE && 
                    xQueuePeek(ElectricalMeasurements_queue, &ElectricalMeasurements_data, pdMS_TO_TICKS(0)) == pdTRUE && 
                    xQueuePeek(TemperatureReadings_queue, &TemperatureReadings_data, pdMS_TO_TICKS(0)) == pdTRUE &&
                    xQueuePeek(TimerData_queue, &TimerData_data, pdMS_TO_TICKS(0)) == pdTRUE)
                {
                    eTask_User_state = UPDATE_BOILER_SETTINGS;
                }
                else
                {
                    eTask_User_state = RECEIVE;

                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                break;

            case UPDATE_BOILER_SETTINGS:
                Update_Boiler_Settings(eLCD_state, &BoilerSettings_data, TimerData_data.second_number, &second_number_lcd_tmp);

                eTask_User_state = END_TIME;

            break;

            case END_TIME:
                End_Time(BoilerSettings_data, ElectricalMeasurements_data, TemperatureReadings_data, &hours_tmp, &minutes_tmp);

                eTask_User_state = UPDATE_SCREEN;

                break;

            case UPDATE_SCREEN:
                Update_Screen(&eLCD_state, BoilerSettings_data, ElectricalMeasurements_data, TemperatureReadings_data, TimerData_data.second_number, &second_number_lcd_tmp, TimerData_data.energy_j, hours_tmp, minutes_tmp);

                eTask_User_state = SEND;

                break;

            case SEND:
                xQueueOverwrite(BoilerSettings_queue, &BoilerSettings_data);

                eTask_User_state = RECEIVE;

                vTaskDelay(pdMS_TO_TICKS(10));

                break;

            default:
                eTask_User_state = RECEIVE;

                vTaskDelay(pdMS_TO_TICKS(10));

                break;
        }
    }
}   