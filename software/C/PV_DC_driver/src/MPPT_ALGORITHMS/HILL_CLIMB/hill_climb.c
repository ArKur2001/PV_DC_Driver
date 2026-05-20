#include "inttypes.h"
#include "hill_climb.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include "data_structures.h"
#include "math.h"

enum MPPT_stage {SETUP, SET_LOWER_DUTY, CHECK_LOWER, SET_HIGHER_DUTY, CHECK_HIGHER, DUTY_SELECT};

void Hill_Climb_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status)
{
    static enum MPPT_stage eMPPT_stage = SETUP;

    static uint8_t pwm_duty_resolution_bit_temp = 0;

    static uint16_t pwm_duty_mppt_lower = 0;
    static uint16_t pwm_duty_mppt_higher = 0;
    static uint16_t pwm_duty_mppt_opt = 0;

    static double power_pwm_lower = 0.0;
    static double power_pwm_higher = 0.0;
    static double power_opt = 0.0;

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
                power_opt = 0.0;
            
                pwm_duty_resolution_bit_temp = pwm_duty_resolution_bit - 1;

                pwm_duty_mppt_lower = pow(2, pwm_duty_resolution_bit_temp) - pow(2, pwm_duty_resolution_bit_temp - 1);
                pwm_duty_mppt_higher = pow(2, pwm_duty_resolution_bit_temp) + pow(2, pwm_duty_resolution_bit_temp - 1);

                eMPPT_stage = SET_LOWER_DUTY;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;
        
        case SET_LOWER_DUTY:
            PWM_set_duty_cycle(pwm_duty_mppt_lower);

            eMPPT_stage = CHECK_LOWER;
            *eTask_MPPT_state = MEASUREMENTS;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

        case CHECK_LOWER:
            power_pwm_lower = ElectricalMeasurements_data.power_value;

            eMPPT_stage = SET_HIGHER_DUTY;
            *eTask_MPPT_state = MPPT;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            
            break;

        case SET_HIGHER_DUTY:
            PWM_set_duty_cycle(pwm_duty_mppt_higher);
            
            eMPPT_stage = CHECK_HIGHER;
            *eTask_MPPT_state = MEASUREMENTS;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

        case CHECK_HIGHER:
            power_pwm_higher = ElectricalMeasurements_data.power_value;    

            eMPPT_stage = DUTY_SELECT;
            *eTask_MPPT_state = MPPT;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;

            break;

        case DUTY_SELECT:
            pwm_duty_resolution_bit_temp--;

            if(power_pwm_lower > power_pwm_higher)
            {
                if(power_pwm_lower > power_opt)
                {
                    power_opt = power_pwm_lower;
                    pwm_duty_mppt_opt = pwm_duty_mppt_lower;
                }
                else
                {
                    power_opt = power_opt;
                    pwm_duty_mppt_opt = pwm_duty_mppt_opt;
                }

                pwm_duty_mppt_lower = pwm_duty_mppt_lower - pow(2, pwm_duty_resolution_bit_temp - 1);
                pwm_duty_mppt_higher = pwm_duty_mppt_lower + pow(2, pwm_duty_resolution_bit_temp - 1);
            }    
            else
            {
                if(power_pwm_higher > power_opt)
                {
                    power_opt = power_pwm_higher;
                    pwm_duty_mppt_opt = pwm_duty_mppt_higher;
                }
                else
                {
                    power_opt = power_opt;
                    pwm_duty_mppt_opt = pwm_duty_mppt_opt;
                }

                pwm_duty_mppt_lower = pwm_duty_mppt_higher - pow(2, pwm_duty_resolution_bit_temp - 1);
                pwm_duty_mppt_higher = pwm_duty_mppt_higher + pow(2, pwm_duty_resolution_bit_temp - 1);
            }

            if(pwm_duty_resolution_bit_temp < 1)
            {
                PWM_set_duty_cycle(pwm_duty_mppt_opt);

                MPPTData_data->power_opt = power_opt;

                eMPPT_stage = SETUP;
                *eTask_MPPT_state = SEND;
                *eAlgorithm_Status = ALGORITHM_DONE;
            }
            else
            {
                eMPPT_stage = SET_LOWER_DUTY;
                *eTask_MPPT_state = MPPT;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;

        default:
            eMPPT_stage = SETUP;

            break;
    }
}