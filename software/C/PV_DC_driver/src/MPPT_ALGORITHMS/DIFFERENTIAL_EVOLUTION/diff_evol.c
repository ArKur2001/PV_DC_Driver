#include "inttypes.h"
#include "diff_evol.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include "data_structures.h"
#include "math.h"
#include "bootloader_random.h"
#include "esp_random.h"

enum MPPT_stage {SETUP, SET_DUTY_I, CHECK_POWER_I, MUTATION_CROSSOVER, SET_DUTY_UI, CHECK_POWER_UI, SELECTION, CHECK_MPP, GET_POWER_MPP};

void generate_random_indices(uint8_t idx[3])
{
    idx[0] = 0;
    idx[1] = 1;
    idx[2] = 2;

    // Fisher-Yates
    for(int i = 2; i > 0; i--)
    {
        int j = esp_random() / (UINT32_MAX / (i + 1) + 1);

        uint8_t temp = idx[i];
        idx[i] = idx[j];
        idx[j] = temp;
    }
}

void Differential_Evolution_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status)
{
    static enum MPPT_stage eMPPT_stage = SETUP;

    uint16_t duty_min = 1;
    uint16_t duty_max = pow(2, pwm_duty_resolution_bit) - 1;

    float F = 0.8;
    float CR = 0.2;

    static float D_i_G[3] = {0.3, 0.6, 0.9};
    static double P_i[3] = {0.0};

    static float DV_i_G[3] = {0.0};
    static float DU_i [3] = {0.0};
    static double PU_i[3] = {0.0};

    static uint16_t idx = 0;

    static float D_best = 0.0;
    static double P_best = 0.0;

    uint16_t duty = 0;

    bootloader_random_enable();

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
                D_i_G[0] = 0.3;
                D_i_G[1] = 0.6;
                D_i_G[2] = 0.9;

                P_i[0] = 0.0;
                P_i[1] = 0.0;   
                P_i[2] = 0.0;

                DV_i_G[0] = 0.0;
                DV_i_G[1] = 0.0;            
                DV_i_G[2] = 0.0;

                DU_i[0] = 0.0;
                DU_i[1] = 0.0;
                DU_i[2] = 0.0;

                PU_i[0] = 0.0;
                PU_i[1] = 0.0;
                PU_i[2] = 0.0;

                idx = 0;

                D_best = 0.0;
                P_best = 0.0;

                eMPPT_stage = SET_DUTY_I;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;
        
        case SET_DUTY_I:
            if(D_i_G[idx] >= 1.0)
            {
                duty = duty_max;
            }
            else if(D_i_G[idx] <= 0.0)
            {
                duty = duty_min;
            }
            else
            {
                duty = duty_min + (uint16_t)(D_i_G[idx] * (duty_max - duty_min));
            }

            PWM_set_duty_cycle(duty);

            eMPPT_stage = CHECK_POWER_I;
            *eTask_MPPT_state = MEASUREMENTS;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

        case CHECK_POWER_I:
            P_i[idx] = ElectricalMeasurements_data.power_value;

            if(P_i[idx] > P_best)
            {
                P_best = P_i[idx];
                D_best = D_i_G[idx];
            }

            idx++;

            if(idx > 2)
            {
                idx = 0;
                eMPPT_stage = MUTATION_CROSSOVER;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }
            else
            {
                eMPPT_stage = SET_DUTY_I;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;

        case MUTATION_CROSSOVER:
            uint8_t i = 0;    
            uint8_t random_idx[3] = {0};
            float Dr[3] = {0.0};

            for(i = 0 ; i < 3 ; i++)
            {
                generate_random_indices(random_idx);

                Dr[0] = D_i_G[random_idx[0]];
                Dr[1] = D_i_G[random_idx[1]];          
                Dr[2] = D_i_G[random_idx[2]];

                if(Dr[0] >= D_best)
                {
                    DV_i_G[i] = Dr[0] - (F * fabsf(Dr[1] - Dr[2]));
                }
                else
                {
                    DV_i_G[i] = Dr[0] + (F * fabsf(Dr[1] - Dr[2]));
                }

                if(DV_i_G[i] >= 1.0)
                {
                    DV_i_G[i] = 1.0;
                }
                else if(DV_i_G[i] <= 0.0)
                {
                    DV_i_G[i] = 0.0;
                }
                else
                {
                    DV_i_G[i] = DV_i_G[i];
                }
            }

            uint8_t j = 0;
            float rand_num = 0.0;

            for(j = 0 ; j < 3 ; j++)
            {
                rand_num = (float)esp_random() / UINT32_MAX;   
                printf("rand_num = %.10f\n", rand_num);
                
                if(rand_num >= (CR/2) && rand_num <= (1 - (CR/2)))
                {
                    DU_i[j] = DV_i_G[j];
                }
                else
                {
                    DU_i[j] = D_i_G[j];
                }
            }

            eMPPT_stage = SET_DUTY_UI;
            *eTask_MPPT_state = MPPT;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

            case SET_DUTY_UI:
                if(DU_i[idx] >= 1.0)
                {
                    duty = duty_max;
                }
                else if(DU_i[idx] <= 0.0)
                {
                    duty = duty_min;
                }
                else
                {
                    duty = duty_min + (uint16_t)(DU_i[idx] * (duty_max - duty_min));
                }

                PWM_set_duty_cycle(duty);

                eMPPT_stage = CHECK_POWER_UI;
                *eTask_MPPT_state = MEASUREMENTS;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

            case CHECK_POWER_UI:
                PU_i[idx] = ElectricalMeasurements_data.power_value;

                idx++;

                if(idx > 2)
                {
                    idx = 0;
                    eMPPT_stage = SELECTION;
                    *eTask_MPPT_state = MPPT;
                    *eAlgorithm_Status = ALGORITHM_NOT_DONE;
                }
                else
                {
                    eMPPT_stage = SET_DUTY_UI;
                    *eTask_MPPT_state = MPPT;
                    *eAlgorithm_Status = ALGORITHM_NOT_DONE;
                }

            break;

            case SELECTION:
                bool improved = false;
                uint8_t k = 0;

                for(k = 0 ; k < 3 ; k++)
                {
                    if(PU_i[k] > P_i[k])
                    {
                        D_i_G[k] = DU_i[k];
                        P_i[k] = PU_i[k];

                        improved = true;
                    }
                    else
                    {
                        D_i_G[k] = D_i_G[k];
                        P_i[k] = P_i[k];
                    }
                }

                P_best = 0.0;
                uint8_t l = 0;

                for(l = 0 ; l < 3 ; l++)
                {
                    if(P_i[l] > P_best)
                    {
                        P_best = P_i[l];
                        D_best = D_i_G[l];
                    }
                }

                if(D_best >= 1.0)
                {
                    duty = duty_max;
                }
                else if(D_best <= 0.0)
                {
                    duty = duty_min;
                }
                else
                {
                    duty = duty_min + (uint16_t)(D_best * (duty_max - duty_min));
                }

                PWM_set_duty_cycle(duty);

                vTaskDelay(pdMS_TO_TICKS(1000)); //delay for the system to stabilize at the new duty cycle

                if(improved == false)
                {
                    eMPPT_stage = GET_POWER_MPP;
                    *eTask_MPPT_state = MEASUREMENTS;
                    *eAlgorithm_Status = ALGORITHM_NOT_DONE;
                }
                else
                {
                    eMPPT_stage = CHECK_MPP;
                    *eTask_MPPT_state = MPPT;
                    *eAlgorithm_Status = ALGORITHM_NOT_DONE;
                }

            break;

        case CHECK_MPP:
            uint8_t m = 0;
            double power_min = P_i[m];
            double power_max = P_i[m];

            for(m = 1 ; m < 3 ; m++)
            {
                if(P_i[m] > power_max)
                {
                    power_max = P_i[m];
                }

                if(P_i[m] < power_min)
                {
                    power_min = P_i[m];
                }
            }

            if((fabs(power_max - power_min)) < 5)
            {
                eMPPT_stage = GET_POWER_MPP;
                *eTask_MPPT_state = MEASUREMENTS;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }
            else
            {
                eMPPT_stage = MUTATION_CROSSOVER;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;        

            case GET_POWER_MPP:
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