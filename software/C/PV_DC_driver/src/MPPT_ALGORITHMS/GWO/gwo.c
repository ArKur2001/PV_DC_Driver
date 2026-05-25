#include "inttypes.h"
#include "gwo.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include "data_structures.h"
#include "math.h"
#include "esp_random.h"

#define N 6 //number of wolves
#define T 5 //iterations limit

enum MPPT_stage {SETUP, SET_DUTY, CHECK_POWER, UPDATE_WOLVES, CHECK_CONVERGENCE, GET_POWER_OPT};

void GWO_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status)
{
    static enum MPPT_stage eMPPT_stage = SETUP;

    uint16_t duty_min = 1;
    uint16_t duty_max = pow(2, pwm_duty_resolution_bit) - 1;

    static uint8_t iteration_number = 1;
    static uint8_t grey_wolf_number = 0;
    static float grey_wolf_duty[N] = {0.0};
    static double grey_wolf_power[N] = {0.0};
    
    int16_t duty = 0;

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
                iteration_number = 1;
                grey_wolf_number = 0;

                int i = 0;
                for(i = 0 ; i < N ; i++)
                {
                    grey_wolf_duty[i] = (float)esp_random() / UINT32_MAX; //random duty cycle between 0 and 1
                    grey_wolf_power[i] = 0.0;

                    vTaskDelay(pdMS_TO_TICKS(1)); //delay to ensure different random numbers for each particle
                }

                eMPPT_stage = SET_DUTY;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;
        
        case SET_DUTY:
            if(grey_wolf_duty[grey_wolf_number] >= 1.0)
            {
                duty = duty_max;
            }
            else if(grey_wolf_duty[grey_wolf_number] <= 0.0)
            {
                duty = duty_min;
            }
            else
            {
                duty = duty_min + (uint16_t)(grey_wolf_duty[grey_wolf_number] * (duty_max - duty_min));
            }

            PWM_set_duty_cycle(duty);

            eMPPT_stage = CHECK_POWER;
            *eTask_MPPT_state = MEASUREMENTS;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

        case CHECK_POWER:
            grey_wolf_power[grey_wolf_number] = ElectricalMeasurements_data.power_value;

            grey_wolf_number++;

            if(grey_wolf_number >= N)
            {
                grey_wolf_number = 0;
                eMPPT_stage = UPDATE_WOLVES;
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

        case UPDATE_WOLVES:
            float a = 2.0 - ((2.0 * (float)iteration_number) / (float)T);
            
            int j = 0;
            int k = 0;
            double temp_power = 0.0;
            float temp_duty = 0.0;

            for (j = 0; j < N - 1; j++)
            {
                for (k = j + 1; k < N; k++)
                {
                    if (grey_wolf_power[j] < grey_wolf_power[k])
                    {
                        temp_power = grey_wolf_power[j];
                        grey_wolf_power[j] = grey_wolf_power[k];
                        grey_wolf_power[k] = temp_power;
                    
                        temp_duty = grey_wolf_duty[j];
                        grey_wolf_duty[j] = grey_wolf_duty[k];
                        grey_wolf_duty[k] = temp_duty;
                    }
                }
            }

            float grey_wolf_duty_buf[N] = {0.0};
            
            int l = 0;
            for(l = 0 ; l < N ; l++)
            {
                float X[3] = {0.0};

                int m = 0;
                for(m = 0 ; m < 3 ; m++)
                {
                    float r1 = (float)esp_random() / UINT32_MAX;
                    vTaskDelay(pdMS_TO_TICKS(1)); //delay to ensure different random numbers for each particle
                    float r2 = (float)esp_random() / UINT32_MAX;
                    vTaskDelay(pdMS_TO_TICKS(1)); //delay to ensure different random numbers for each particle

                    float A = (2 * a * r1) - a;
                    float C = 2 * r2;

                    float D = fabs((C * grey_wolf_duty[m]) - grey_wolf_duty[l]);
                    X[m] = grey_wolf_duty[m] - (A * D);
                }

                grey_wolf_duty_buf[l] = (X[0] + X[1] + X[2]) / 3;

                if(grey_wolf_duty_buf[l] > 1.0)
                {
                    grey_wolf_duty_buf[l] = 1.0;
                }
                else if(grey_wolf_duty_buf[l] < 0.0)
                {
                    grey_wolf_duty_buf[l] = 0.0;
                }
                else
                {
                    grey_wolf_duty_buf[l] = grey_wolf_duty_buf[l];
                }
            }

            int n = 0;
            for(n = 0 ; n < N ; n++)
            {
                grey_wolf_duty[n] = grey_wolf_duty_buf[n];
                grey_wolf_power[n] = 0.0;
            }

             eMPPT_stage = CHECK_CONVERGENCE;
             *eTask_MPPT_state = MPPT;
             *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

            case CHECK_CONVERGENCE:
                iteration_number++;

                if (iteration_number > T)
                {
                    iteration_number = 1;

                    duty = duty_min + (uint16_t)(grey_wolf_duty[0] * (duty_max - duty_min));
                    PWM_set_duty_cycle(duty);

                    vTaskDelay(pdMS_TO_TICKS(1000)); //delay to ensure stable power measurement

                    eMPPT_stage = GET_POWER_OPT;
                    *eTask_MPPT_state = MEASUREMENTS;
                    *eAlgorithm_Status = ALGORITHM_NOT_DONE;
                }
                else
                {
                    iteration_number = iteration_number;

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