#include "inttypes.h"
#include "no_algorithm.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include "data_structures.h"
#include "math.h"

void No_algorithm(MPPTData *MPPTData_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status)
{
    if(MPPTData_data->eMPPT_Permission == MPPT_NOT_ALLOWED)
    {
        *eTask_MPPT_state = SEND;
        *eAlgorithm_Status = ALGORITHM_DONE;
    }
    else
    {
        PWM_set_duty_cycle((pow(2, pwm_duty_resolution_bit)) - 1);
        *eTask_MPPT_state = SEND;
        *eAlgorithm_Status = ALGORITHM_NOT_DONE;
    }
}