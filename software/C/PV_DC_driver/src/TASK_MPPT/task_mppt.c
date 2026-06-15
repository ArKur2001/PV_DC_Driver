#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include <ADC/adc.h>
#include <MEASUREMENTS/measurements.h>
#include "data_structures.h"
#include "math.h"

//#include <MPPT_ALGORITHMS/NO_ALGORITHM/no_algorithm.h>
//#include <MPPT_ALGORITHMS/HILL_CLIMB/hill_climb.h>
//#include <MPPT_ALGORITHMS/P_and_O/p_and_o.h>
//#include <MPPT_ALGORITHMS/THREE_POINT_WEIGHT/tpw.h>
//#include <MPPT_ALGORITHMS/CURRENT_SWEEP/current_sweep.h>
//#include <MPPT_ALGORITHMS/DIFFERENTIAL_EVOLUTION/diff_evol.h>
//#include <MPPT_ALGORITHMS/INCREMENTAL_CONDUCTANCE/inc_cond.h>
//#include <MPPT_ALGORITHMS/PSO/pso.h>
//#include <MPPT_ALGORITHMS/GWO/gwo.h>
#include <MPPT_ALGORITHMS/FUZZY_LOGIC/fuzzy_logic.h>
//#include <MPPT_ALGORITHMS/PSO_P&O/hybrid.h>

#define ADC_SAMPLES_NUMBER          100 
#define MEASUREMENT_DELAY           500     //mimimum 3 time constants (ms)

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
    
    //printf("duty_cycle = %" PRIu8 "\n", PWM_get_duty_cycle());
    //printf("Voltage RMS value = %f V\n", ElectricalMeasurements_data->voltage_value);
    //printf("Current RMS value = %f A\n", ElectricalMeasurements_data->current_value); 
    //printf("Power RMS value = %f W\n", ElectricalMeasurements_data->power_value);    
    
    printf("%.6f\n", ElectricalMeasurements_data->power_value);
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
                //No_algorithm(&MPPTData_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status); //DO NOT USE THIS ALGORITHM IN REAL APPLICATION, TEMPERATURE READING ISN'T WORKING !!!
                //Hill_Climb_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status);
                //P_and_O_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status); //DO NOT USE THIS ALGORITHM IN REAL APPLICATION, TEMPERATURE READING ISN'T WORKING !!!
                //TPW_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status); //DO NOT USE THIS ALGORITHM IN REAL APPLICATION, TEMPERATURE READING ISN'T WORKING !!!
                //Current_Sweep_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status);
                //Differential_Evolution_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status);
                //Incremental_Conductance_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status);
                //PSO_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status);
                //GWO_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status);
                Fuzzy_logic_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status); //DO NOT USE THIS ALGORITHM IN REAL APPLICATION, TEMPERATURE READING ISN'T WORKING !!!
                //Hybrid_algorithm(&MPPTData_data, ElectricalMeasurements_data, pwm_duty_resolution_bit, &eTask_MPPT_state, &eAlgorithm_Status); //DO NOT USE THIS ALGORITHM IN REAL APPLICATION, TEMPERATURE READING ISN'T WORKING !!!

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