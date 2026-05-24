#include "inttypes.h"
#include "pso.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include "data_structures.h"
#include "math.h"
#include "esp_random.h"

#define N 5 //number of particles

enum MPPT_stage {SETUP, SET_DUTY, CHECK_POWER, UPDATE_VEL_POS, CHECK_CONVERGENCE, GET_POWER_OPT};

void PSO_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status)
{
    static enum MPPT_stage eMPPT_stage = SETUP;

    uint16_t duty_min = 1;
    uint16_t duty_max = pow(2, pwm_duty_resolution_bit) - 1;

    float w = 0.6; //inertia weight
    float c1 = 1.2; //personal learning coefficient
    float c2 = 2.0; //global learning coefficient

    uint8_t convergence_limit = 5;

    static uint8_t convergence_counter = 0;
    static uint8_t particle_number = 0;
    static float particle_duty[N] = {0.0};
    static float particle_best_duty[N] = {0.0};
    static float global_best_duty = 0.0;
    static double particle_best_power[N] = {0.0};
    static double global_best_power = 0.0;
    static double global_best_power_prev = 0.0;
    static float particle_velocity[N] = {0.0};

    uint16_t duty = 0;

    switch (eMPPT_stage)
    {
        case SETUP:
            if(MPPTData_data->eMPPT_Permission == MPPT_NOT_ALLOWED)
            {
                eMPPT_stage = SETUP;
                *eTask_MPPT_state = SEND;
                *eAlgorithm_Status = ALGORITHM_DONE;
            }
            else
            {
                int i = 0;
                for(i = 0 ; i < N ; i++)
                {
                    particle_duty[i] = (float)esp_random() / UINT32_MAX; //random duty cycle between 0 and 1
                    particle_best_duty[i] = 0.0;
                    particle_best_power[i] = 0.0;
                    particle_velocity[i] = 0.0;

                    vTaskDelay(pdMS_TO_TICKS(1)); //delay to ensure different random numbers for each particle
                }
                
                particle_number = 0;
                global_best_duty = 0.0;
                global_best_power = 0.0;
                global_best_power_prev = 0.0;

                eMPPT_stage = SET_DUTY;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;
        
        case SET_DUTY:
            if(particle_duty[particle_number] >= 1.0)
            {
                duty = duty_max;
            }
            else if(particle_duty[particle_number] <= 0.0)
            {
                duty = duty_min;
            }
            else
            {
                duty = duty_min + (uint16_t)(particle_duty[particle_number] * (duty_max - duty_min));
            }

            PWM_set_duty_cycle(duty);

            eMPPT_stage = CHECK_POWER;
            *eTask_MPPT_state = MEASUREMENTS;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

        case CHECK_POWER:
            if(ElectricalMeasurements_data.power_value > particle_best_power[particle_number])
            {
                particle_best_power[particle_number] = ElectricalMeasurements_data.power_value;
                particle_best_duty[particle_number] = particle_duty[particle_number];

                if(ElectricalMeasurements_data.power_value > global_best_power)
                {
                    global_best_power = ElectricalMeasurements_data.power_value;
                    global_best_duty = particle_duty[particle_number];
                }
            }

            particle_number++;

            if(particle_number >= N)
            {
                particle_number = 0;
                eMPPT_stage = UPDATE_VEL_POS;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }
            else
            {
                eMPPT_stage = SET_DUTY;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;

        case UPDATE_VEL_POS:
            uint8_t j = 0;
            
            for(j = 0 ; j < N ; j++)
            {
                float r1 = (float)esp_random() / UINT32_MAX;
                vTaskDelay(pdMS_TO_TICKS(1)); //delay to ensure different random numbers for each particle
                float r2 = (float)esp_random() / UINT32_MAX;
                vTaskDelay(pdMS_TO_TICKS(1)); //delay to ensure different random numbers for each particle

                particle_velocity[j] = (w * particle_velocity[j]) + (c1 * r1 * (particle_best_duty[j] - particle_duty[j])) + (c2 * r2 * (global_best_duty - particle_duty[j]));
            }

            uint8_t k = 0;
            for(k = 0 ; k < N ; k++)
            {
                particle_duty[k] = particle_duty[k] + particle_velocity[k];

                if(particle_duty[k] > 1.0)
                {
                    particle_duty[k] = 1.0;
                }
                else if(particle_duty[k] < 0.0)
                {
                    particle_duty[k] = 0.0;
                }
                else
                {
                    particle_duty[k] = particle_duty[k];
                }
            }

             eMPPT_stage = CHECK_CONVERGENCE;
             *eTask_MPPT_state = MPPT;
             *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

            case CHECK_CONVERGENCE:
                if (fabs(global_best_power - global_best_power_prev) < 1.0)
                {
                    convergence_counter++;
                }
                else
                {
                    convergence_counter = 0;
                }

                global_best_power_prev = global_best_power;

                if(convergence_counter >= convergence_limit)
                {
                    duty = duty_min + (uint16_t)(global_best_duty * (duty_max - duty_min));
                    PWM_set_duty_cycle(duty);

                    vTaskDelay(pdMS_TO_TICKS(1000)); //delay for the system to stabilize at the new duty cycle

                    eMPPT_stage = GET_POWER_OPT;
                    *eTask_MPPT_state = MEASUREMENTS;
                    *eAlgorithm_Status = ALGORITHM_NOT_DONE;
                }
                else
                {
                    eMPPT_stage = SET_DUTY;
                    *eTask_MPPT_state = MPPT;
                    *eAlgorithm_Status = ALGORITHM_NOT_DONE;
                }

            break;

            case GET_POWER_OPT:
                MPPTData_data->power_opt = ElectricalMeasurements_data.power_value;

                eMPPT_stage = SETUP;
                *eTask_MPPT_state = SEND;
                *eAlgorithm_Status = ALGORITHM_DONE;

            break;

        default:
            eMPPT_stage = SETUP;

            break;
    }
}