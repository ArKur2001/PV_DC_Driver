#include "inttypes.h"
#include "tpw.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include "data_structures.h"
#include "math.h"

enum MPPT_stage {SETUP, SET_DUTY, CHECK_POWER, DUTY_SELECT};

void TPW_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status)
{
    static enum MPPT_stage eMPPT_stage = SETUP;

    uint16_t duty_min = 1;
    uint16_t duty_max = pow(2, pwm_duty_resolution_bit) - 1;

    static int16_t duty[3] = {0};
    static double power[3] = {0.0};
  
    int8_t wAB = 0;
    int8_t wAC = 0;
    int8_t sumW = 0;

    static uint8_t counter = 0;

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
                duty[1] = pow(2, pwm_duty_resolution_bit - 1);
                duty[0] = duty[1] - 1;
                duty[2] = duty[1] + 1;
            
                power[0] = 0.0;
                power[1] = 0.0;
                power[2] = 0.0;

                counter = 0;

                eMPPT_stage = SET_DUTY;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;
        
        case SET_DUTY:
            PWM_set_duty_cycle(duty[counter]);

            eMPPT_stage = CHECK_POWER;
            *eTask_MPPT_state = MEASUREMENTS;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

        case CHECK_POWER:
            power[counter] = ElectricalMeasurements_data.power_value;

            counter++;

            if(counter > 2)
            {
                counter = 0;
                eMPPT_stage = DUTY_SELECT;
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

        case DUTY_SELECT:
            if(power[2] >= power[1])
            {
                wAB = 1;
            }
            else
            {
                wAB = -1;
            }

            if(power[0] < power[1])
            {
                wAC = 1;
            }
            else
            {
                wAC = -1;
            }

            sumW = wAB + wAC;

            if(sumW == 2)
            {
                duty[1] = duty[1] + 1;
            }
            else if(sumW == -2)
            {
                duty[1] = duty[1] - 1;
            }
            else
            {
                duty[1] = duty[1];
            }

            duty[0] = duty[1] - 1;
            duty[2] = duty[1] + 1;

            if(duty[0] >= duty_max)
            {
                duty[0] = duty_max;
            }
            else if(duty[0] <= duty_min)
            {
                duty[0] = duty_min;
            }

            if(duty[1] >= duty_max)
            {
                duty[1] = duty_max;
            }
            else if(duty[1] <= duty_min)
            {
                duty[1] = duty_min;
            }

            if(duty[2] >= duty_max)
            {
                duty[2] = duty_max;
            }
            else if(duty[2] <= duty_min)
            {
                duty[2] = duty_min;
            }

            PWM_set_duty_cycle(duty[counter]);

            eMPPT_stage = CHECK_POWER;
            *eTask_MPPT_state = SEND;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            
            break;

        default:
            eMPPT_stage = SETUP;

            break;
    }
}