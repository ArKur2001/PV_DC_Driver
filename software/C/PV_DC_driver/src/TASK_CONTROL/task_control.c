#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "TASK_CONTROL/task_control.h"
#include "math.h"
#include "DS18B20/ds18b20.h"
#include "LED/led.h"
#include "PWM/pwm.h"
#include "data_structures.h"

#define CASE_MAX_TEMP                   75
#define ERROR_TEMP                      0
#define TEMPERATURE_MEASUREMENT_PERIOD  10  //s

#define MPPT_PERIOD                     30  //s
#define MPPT_POWER_DIFF                 50

enum Task_Control_state     {RECEIVE, READ_TEMP, HEATING_CONTROL, MPPT_CONTROL, SET_LED, SEND};
enum Water_Heating_Status   {STOP_HEATING, ALLOW_HEATING, ERROR};

void Read_Temp(TemperatureReadings *TemperatureReadings_data, enum Water_Heating_Status eWater_Heating_Status, uint8_t boiler_sensor_pin, uint8_t case_sensor_pin, uint64_t second_number)
{
    static uint64_t second_number_temp = 0;

    if(second_number_temp <= second_number || eWater_Heating_Status != ALLOW_HEATING)
    {
        bool pwm_was_on = false;
        
        if(PWM_get_state() == PWM_ON)
        {
            pwm_was_on = true;
        }
        else
        {
            pwm_was_on = false;
        }

        if (pwm_was_on == true)
        {
            PWM_control(PWM_OFF);
        }

        TemperatureReadings_data->temp_water = ds18b20_get_temp(boiler_sensor_pin);
        TemperatureReadings_data->temp_case = ds18b20_get_temp(case_sensor_pin);

        second_number_temp = second_number + TEMPERATURE_MEASUREMENT_PERIOD;

        if (pwm_was_on == true)
        {
            PWM_control(PWM_ON);
        }
    }  
}

void Heating_Control(TemperatureReadings TemperatureReadings_data, BoilerSettings BoilerSettings_data, enum Water_Heating_Status *eWater_Heating_Status)
{
    if(TemperatureReadings_data.temp_case >= CASE_MAX_TEMP || TemperatureReadings_data.temp_case <= ERROR_TEMP  || TemperatureReadings_data.temp_water <= ERROR_TEMP)
    {
        PWM_control(PWM_OFF);

        *eWater_Heating_Status = ERROR;
    }
    else if(*eWater_Heating_Status != ALLOW_HEATING && TemperatureReadings_data.temp_water >= BoilerSettings_data.desired_temperature - 1)
    {
        PWM_control(PWM_OFF);

        *eWater_Heating_Status = STOP_HEATING;
    }
    else if(TemperatureReadings_data.temp_water >= BoilerSettings_data.desired_temperature)
    {
        PWM_control(PWM_OFF);

        *eWater_Heating_Status = STOP_HEATING;
    }
    else
    {
        PWM_control(PWM_ON);

        *eWater_Heating_Status = ALLOW_HEATING;
    }
}

void MPPT_control(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, enum Water_Heating_Status eWater_Heating_Status, uint64_t second_number)
{
    static enum Water_Heating_Status eWater_Heating_Status_previous = STOP_HEATING;
    static uint64_t second_number_mppt = 0;

    double power_diff = 0.0;
    
    power_diff = fabs(MPPTData_data->power_opt - ElectricalMeasurements_data.power_value);

    if((eWater_Heating_Status == ALLOW_HEATING) && ((second_number_mppt <= second_number || power_diff >= MPPT_POWER_DIFF) || eWater_Heating_Status_previous != ALLOW_HEATING))
    {
        MPPTData_data->eMPPT_Permission = MPPT_ALLOWED;

        second_number_mppt = second_number + MPPT_PERIOD;
    }
    else
    {
        MPPTData_data->eMPPT_Permission = MPPT_NOT_ALLOWED;
    }

    eWater_Heating_Status_previous = eWater_Heating_Status;
}

void Set_Led(enum Water_Heating_Status eWater_Heating_Status)
{
    switch (eWater_Heating_Status)
    {
    case STOP_HEATING:
        Led_green_set(LED_ON);
        Led_red_set(LED_ON);

        break;
    
    case ALLOW_HEATING:
        Led_green_set(LED_ON);
        Led_red_set(LED_OFF);
        break;

    case ERROR:
        Led_green_set(LED_OFF);
        Led_red_set(LED_ON);
        break;

    default:
        Led_green_set(LED_OFF);
        Led_red_set(LED_OFF);

        break;
    }
}

void Task_Control(void *pvParameters)
{
    TaskControlParameters *params = (TaskControlParameters *)pvParameters;

    QueueHandle_t BoilerSettings_queue = params->BoilerSettings_queue;
    QueueHandle_t ElectricalMeasurements_queue = params->ElectricalMeasurements_queue;
    QueueHandle_t TemperatureReadings_queue = params->TemperatureReadings_queue;
    QueueHandle_t TimerData_queue = params->TimerData_queue;
    QueueHandle_t MPPTData_queue = params->MPPTData_queue;
    uint8_t boiler_sensor_pin = params->boiler_sensor_pin;
    uint8_t case_sensor_pin = params->case_sensor_pin;

    static enum Task_Control_state eTask_Control_state = RECEIVE;
    static enum Water_Heating_Status eWater_Heating_Status = STOP_HEATING;

    static BoilerSettings BoilerSettings_data = {50, 100};
    static ElectricalMeasurements ElectricalMeasurements_data = {0.0, 0.0, 0.0};
    static TemperatureReadings TemperatureReadings_data = {0.0, 0.0};
    static TimerData TimerData_data = {0, 0};
    static MPPTData MPPTData_data = {0.0, MPPT_NOT_ALLOWED};


    while(1)
    {
        switch (eTask_Control_state)
        {
        case RECEIVE:
            if (xQueuePeek(BoilerSettings_queue, &BoilerSettings_data, pdMS_TO_TICKS(0)) == pdTRUE && 
                xQueuePeek(ElectricalMeasurements_queue, &ElectricalMeasurements_data, pdMS_TO_TICKS(0)) == pdTRUE && 
                xQueuePeek(TemperatureReadings_queue, &TemperatureReadings_data, pdMS_TO_TICKS(0)) == pdTRUE &&
                xQueuePeek(TimerData_queue, &TimerData_data, pdMS_TO_TICKS(0)) == pdTRUE &&
                xQueuePeek(MPPTData_queue, &MPPTData_data, pdMS_TO_TICKS(0)) == pdTRUE)
            {
                if(MPPTData_data.eMPPT_Permission == MPPT_ALLOWED)
                {
                    eTask_Control_state = RECEIVE;

                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                else
                {
                    eTask_Control_state = READ_TEMP;
                }
            }
            else
            {
                eTask_Control_state = RECEIVE;

                vTaskDelay(pdMS_TO_TICKS(10));
            }

            break;

        case READ_TEMP:
            Read_Temp(&TemperatureReadings_data, eWater_Heating_Status, boiler_sensor_pin, case_sensor_pin, TimerData_data.second_number);

            eTask_Control_state = HEATING_CONTROL;
            break;

        case HEATING_CONTROL:
            Heating_Control(TemperatureReadings_data, BoilerSettings_data, &eWater_Heating_Status);

            eTask_Control_state = MPPT_CONTROL;

            break;

        case MPPT_CONTROL:
            MPPT_control(&MPPTData_data, ElectricalMeasurements_data, eWater_Heating_Status, TimerData_data.second_number);

            eTask_Control_state = SET_LED;

            break;

        case SET_LED:
            Set_Led(eWater_Heating_Status);

            eTask_Control_state = SEND;

            break;

        case SEND:
            xQueueOverwrite(TemperatureReadings_queue, &TemperatureReadings_data);
            xQueueOverwrite(MPPTData_queue, &MPPTData_data);

            eTask_Control_state = RECEIVE; 

            vTaskDelay(pdMS_TO_TICKS(10));

            break;
        
        default:
            eTask_Control_state = RECEIVE;

            vTaskDelay(pdMS_TO_TICKS(10));
            
            break;
        }
    }
}