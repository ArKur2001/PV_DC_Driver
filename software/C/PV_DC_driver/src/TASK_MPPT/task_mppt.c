#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include <ADC/adc.h>
#include <MEASUREMENTS/measurements.h>
#include "data_structures.h"
#include "math.h"

#include <MPPT_ALGORITHMS/HILL_CLIMB/hill_climb.h>
//#include <MPPT_ALGORITHMS/NO_ALGORITHM/no_algorithm.h>

#define ADC_SAMPLES_NUMBER          100 
#define MEASUREMENT_DELAY           100     //mimimum 3 time constants (ms)

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

void Task_MPPT(void *pvParameters)
{
    TaskMPPTParameters *params = (TaskMPPTParameters *)pvParameters;

    QueueHandle_t ElectricalMeasurements_queue = params->ElectricalMeasurements_queue;
    QueueHandle_t MPPTData_queue = params->MPPTData_queue;
    uint8_t adc_voltage_pin = params->adc_voltage_pin;
    uint8_t adc_current_pin = params->adc_current_pin;
    uint8_t pwm_duty_resolution_bit = params->pwm_duty_resolution_bit;

    static enum Task_MPPT_state eTask_MPPT_state = RECEIVE;
    static enum Algorithm_Status eAlgorithm_Status = ALGORITHM_DONE;

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
                Hill_Climb_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status);
                //No_algorithm(&MPPTData_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status); //DO NOT USE THIS ALGORITHM IN REAL APPLICATION, TEMPERATURE READING ISN'T WORKING !!!

                vTaskDelay(pdMS_TO_TICKS(10));

                break;

            case SEND:
                if(MPPTData_data.eMPPT_Permission == MPPT_ALLOWED && eAlgorithm_Status == ALGORITHM_DONE)
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