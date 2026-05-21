#include "inttypes.h"
#include "current_sweep.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include "data_structures.h"
#include "math.h"

enum MPPT_stage {SETUP, SET_DUTY, CHECK_POWER, CHECK_MPP};

void Current_Sweep_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status)
{
    static enum MPPT_stage eMPPT_stage = SETUP;

    uint16_t duty_min = 1;
    uint16_t duty_max = pow(2, pwm_duty_resolution_bit) - 1;

    static int16_t duty = 0;
    static int16_t duty_mpp = 0;
    static double power_mpp = 0.0;

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
                duty = duty_min;
                duty_mpp = 0;
                power_mpp = 0.0;

                eMPPT_stage = SET_DUTY;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;
        
        case SET_DUTY:
            PWM_set_duty_cycle(duty);

            eMPPT_stage = CHECK_POWER;
            *eTask_MPPT_state = MEASUREMENTS;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

        case CHECK_POWER:
            if(ElectricalMeasurements_data.power_value > power_mpp)
            {
                power_mpp = ElectricalMeasurements_data.power_value;
                duty_mpp = duty;
            }

            duty = duty + 1;

            if(duty >= duty_max)
            {
                PWM_set_duty_cycle(duty_mpp);

                vTaskDelay(pdMS_TO_TICKS(1000)); //delay for the system to stabilize at the new duty cycle

                eMPPT_stage = CHECK_MPP;
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

        case CHECK_MPP:
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