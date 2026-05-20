#include "inttypes.h"
#include "p_and_o.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include "data_structures.h"
#include "math.h"

enum MPPT_stage {SETUP, SET_DUTY};

void P_and_O_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status)
{
    static enum MPPT_stage eMPPT_stage = SETUP;

    static int8_t direction = 1;

    uint16_t duty_min = 1;
    uint16_t duty_max = pow(2, pwm_duty_resolution_bit) - 1;
    static int16_t duty = 0;

    static double power_prev = 0.0;
    double dP = 0.0;

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
                direction = 1;

                duty = pow(2, pwm_duty_resolution_bit - 1);

                power_prev = 0.0;

                eMPPT_stage = SET_DUTY;
                *eTask_MPPT_state = MEASUREMENTS;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;
        
        case SET_DUTY:
            dP = ElectricalMeasurements_data.power_value - power_prev;
            
            if(dP < 0.0)
            {
                direction = -direction;
            }

            duty = duty + direction;

            if(duty >= duty_max)
            {
                duty = duty_max;
                direction = -direction;
            }
            else if(duty <= duty_min)
            {
                duty = duty_min;
                direction = -direction;
            }

            power_prev = ElectricalMeasurements_data.power_value;

            PWM_set_duty_cycle(duty);

            
            eMPPT_stage = SET_DUTY;
            *eTask_MPPT_state = SEND;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            
            break;

        default:
            eMPPT_stage = SETUP;

            break;
    }
}