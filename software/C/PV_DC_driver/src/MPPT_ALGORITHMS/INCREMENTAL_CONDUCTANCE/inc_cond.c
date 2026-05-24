#include "inttypes.h"
#include "inc_cond.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include "data_structures.h"
#include "math.h"

enum MPPT_stage {SETUP, SET_DUTY, DECIDE};

void Incremental_Conductance_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status)
{
    static enum MPPT_stage eMPPT_stage = SETUP;

    uint16_t duty_min = 1;
    uint16_t duty_max = pow(2, pwm_duty_resolution_bit) - 1;

    static double voltage_prev = 0.0;
    static double current_prev = 0.0;

    double dV = 0.0;
    double dI = 0.0;

    static int16_t duty = 0;

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
                voltage_prev = 0.0;
                current_prev = 0.0;

                duty = pow(2, pwm_duty_resolution_bit - 1);

                eMPPT_stage = SET_DUTY;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;
        
        case SET_DUTY:
            if(duty >= duty_max)
            {
                duty = duty_max;
            }
            else if(duty <= duty_min)
            {
                duty = duty_min;
            }

            PWM_set_duty_cycle(duty);

            eMPPT_stage = DECIDE;
            *eTask_MPPT_state = MEASUREMENTS;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            
            break;

        case DECIDE:
            dV = ElectricalMeasurements_data.voltage_value - voltage_prev;
            dI = ElectricalMeasurements_data.current_value - current_prev;    

            if(fabs(dV) < 0.1)
            {
                if(fabs(dI) < 0.005)
                {
                    duty = duty;
                }
                else
                {
                    if(dI > 0)
                    {
                        duty += 1;
                    }
                    else
                    {
                        duty -= 1;
                    }
                }    
            }
            else
            {
                if(((dI / dV) - (ElectricalMeasurements_data.current_value / ElectricalMeasurements_data.voltage_value)) < 0.1)
                {
                    duty = duty;
                }
                else
                {
                    if((dI / dV) > -(ElectricalMeasurements_data.current_value / ElectricalMeasurements_data.voltage_value))
                    {
                        duty += 1;
                    }
                    else
                    {
                        duty -= 1;
                    }
                }
            }

            voltage_prev = ElectricalMeasurements_data.voltage_value;
            current_prev = ElectricalMeasurements_data.current_value;
            
            eMPPT_stage = SET_DUTY;
            *eTask_MPPT_state = SEND;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

        default:
            eMPPT_stage = SETUP;

            break;
    }
}