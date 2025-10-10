#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include <ADC/adc.h>
#include <MEASUREMENTS/measurements.h>
#include "data_structures.h"
#include "math.h"

#define ADC_SAMPLES_NUMBER          100 
#define MEASUREMENT_DELAY           300     //3 time constants (ms)

enum Task_MPPT_state    {RECEIVE, MEASUREMENTS, MPPT, SEND};
enum MPPT_stage         {SETUP, SET_LOWER_DUTY, CHECK_LOWER, SET_HIGHER_DUTY, CHECK_HIGHER, DUTY_SELECT};

void Measurements(ElectricalMeasurements *ElectricalMeasurements_data, uint8_t adc_voltage_pin, uint8_t adc_current_pin, uint8_t pwm_duty_resolution_bit)
{
    vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_DELAY));

    if(PWM_get_state() == PWM_ON && PWM_get_duty_cycle() != 0)
    {
        uint16_t pwm_duty_resolution = pow(2, pwm_duty_resolution_bit) - 1;

        ElectricalMeasurements_data->voltage_value = get_voltage_value(adc_read_voltage(adc_voltage_pin, ADC_SAMPLES_NUMBER), PWM_get_duty_cycle(), pwm_duty_resolution);
        ElectricalMeasurements_data->current_value = get_current_value(adc_read_voltage(adc_current_pin, ADC_SAMPLES_NUMBER), PWM_get_duty_cycle(), pwm_duty_resolution);
        ElectricalMeasurements_data->power_value = ElectricalMeasurements_data->voltage_value * ElectricalMeasurements_data->current_value;
    }
    else
    {
        ElectricalMeasurements_data->voltage_value = get_voltage_value_mean(adc_read_voltage(adc_voltage_pin, ADC_SAMPLES_NUMBER));
        ElectricalMeasurements_data->current_value = get_current_value_mean(adc_read_voltage(adc_current_pin, ADC_SAMPLES_NUMBER));
        ElectricalMeasurements_data->power_value = 0.0;
    }
    
    printf("duty_cycle = %" PRIu8 "\n", PWM_get_duty_cycle());
    printf("Voltage RMS value = %f V\n", ElectricalMeasurements_data->voltage_value);
    printf("Current RMS value = %f A\n", ElectricalMeasurements_data->current_value); 
    printf("Power RMS value = %f W\n", ElectricalMeasurements_data->power_value);    
}

void MPPT_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state)
{
    static enum MPPT_stage eMPPT_stage = SETUP;

    static uint8_t pwm_duty_resolution_bit_temp = 0;

    static uint16_t pwm_duty_mppt_lower = 0;
    static uint16_t pwm_duty_mppt_higher = 0;

    static double power_pwm_lower = 0.0;
    static double power_pwm_higher = 0.0;

    double power_opt = 0.0;

    switch (eMPPT_stage)
    {
        case SETUP:
            pwm_duty_resolution_bit_temp = pwm_duty_resolution_bit - 1;

            if(MPPTData_data->eMPPT_Permission == MPPT_NOT_ALLOWED)
            {
                eMPPT_stage = SETUP;
                *eTask_MPPT_state = SEND;
            }
            else
            {
                pwm_duty_mppt_lower = pow(2, pwm_duty_resolution_bit_temp) - pow(2, pwm_duty_resolution_bit_temp - 1);
                pwm_duty_mppt_higher = pow(2, pwm_duty_resolution_bit_temp) + pow(2, pwm_duty_resolution_bit_temp - 1);

                eMPPT_stage = SET_LOWER_DUTY;
                *eTask_MPPT_state = MPPT;
            }

            break;
        
        case SET_LOWER_DUTY:
            PWM_set_duty_cycle(pwm_duty_mppt_lower);

            eMPPT_stage = CHECK_LOWER;
            *eTask_MPPT_state = MEASUREMENTS;

            break;

        case CHECK_LOWER:
            power_pwm_lower = ElectricalMeasurements_data.power_value;

            eMPPT_stage = SET_HIGHER_DUTY;
            *eTask_MPPT_state = MPPT;
            
            break;

        case SET_HIGHER_DUTY:
            PWM_set_duty_cycle(pwm_duty_mppt_higher);
            
            eMPPT_stage = CHECK_HIGHER;
            *eTask_MPPT_state = MEASUREMENTS;
             
            break;

        case CHECK_HIGHER:
            power_pwm_higher = ElectricalMeasurements_data.power_value;    

            eMPPT_stage = DUTY_SELECT;
            *eTask_MPPT_state = MPPT;

            break;

        case DUTY_SELECT:
            pwm_duty_resolution_bit_temp--;

            if(power_pwm_lower > power_pwm_higher)
            {
                PWM_set_duty_cycle(pwm_duty_mppt_lower);

                power_opt = power_pwm_lower;

                pwm_duty_mppt_lower = pwm_duty_mppt_lower - pow(2, pwm_duty_resolution_bit_temp - 1);
                pwm_duty_mppt_higher = pwm_duty_mppt_lower + pow(2, pwm_duty_resolution_bit_temp - 1);
            }    
            else
            {
                PWM_set_duty_cycle(pwm_duty_mppt_higher);

                power_opt = power_pwm_higher;

                pwm_duty_mppt_lower = pwm_duty_mppt_higher - pow(2, pwm_duty_resolution_bit_temp - 1);
                pwm_duty_mppt_higher = pwm_duty_mppt_higher + pow(2, pwm_duty_resolution_bit_temp - 1);
            }

            if(pwm_duty_resolution_bit_temp < 1)
            {
                MPPTData_data->power_opt = power_opt;

                eMPPT_stage = SETUP;
                *eTask_MPPT_state = SEND;
            }
            else
            {
                eMPPT_stage = SET_LOWER_DUTY;
                *eTask_MPPT_state = MPPT;
            }

            break;

        default:
            eMPPT_stage = SETUP;

            break;
    }
}

void Task_MPPT(void *pvParameters)
{
    TaskMPPTParameters *params = (TaskMPPTParameters *)pvParameters;

    QueueHandle_t ElectricalMeasurements_queue = params->ElectricalMeasurements_queue;
    QueueHandle_t MPPTData_queue = params->MPPTData_queue;
    uint8_t adc_voltage_pin = params->adc_voltage_pin;
    uint8_t adc_current_pin = params->adc_current_pin;
    uint8_t pwm_duty_resolution_bit = params->pwm_duty_resolution_bit;

    static enum Task_MPPT_state eTask_MPPT_state = RECEIVE;

    static ElectricalMeasurements ElectricalMeasurements_data = {0.0, 0.0, 0.0};
    static MPPTData MPPTData_data = {0.0, MPPT_NOT_ALLOWED};

    while(1)
    {
        switch (eTask_MPPT_state)
        {
            case RECEIVE:
                if (xQueuePeek(ElectricalMeasurements_queue, &ElectricalMeasurements_data, pdMS_TO_TICKS(0)) == pdTRUE && 
                    xQueuePeek(MPPTData_queue, &MPPTData_data, pdMS_TO_TICKS(0)) == pdTRUE)
                {
                    eTask_MPPT_state = MEASUREMENTS;

                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                else
                {
                    eTask_MPPT_state = RECEIVE;
                
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                break;

            case MEASUREMENTS:
                Measurements(&ElectricalMeasurements_data, adc_voltage_pin, adc_current_pin, pwm_duty_resolution_bit);

                eTask_MPPT_state = MPPT;

                break;

            case MPPT:
                MPPT_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state);

                vTaskDelay(pdMS_TO_TICKS(10));

                break;

            case SEND:
                if(MPPTData_data.eMPPT_Permission == MPPT_ALLOWED)
                {
                    MPPTData_data.eMPPT_Permission = MPPT_NOT_ALLOWED;

                    xQueueOverwrite(ElectricalMeasurements_queue, &ElectricalMeasurements_data);
                    xQueueOverwrite(MPPTData_queue, &MPPTData_data);
                }
                else
                {
                    xQueueOverwrite(ElectricalMeasurements_queue, &ElectricalMeasurements_data);
                }
               
                eTask_MPPT_state = RECEIVE;

                vTaskDelay(pdMS_TO_TICKS(10));

                break;

            default:
                eTask_MPPT_state = RECEIVE;

                vTaskDelay(pdMS_TO_TICKS(10));

                break;
        }
    }
}