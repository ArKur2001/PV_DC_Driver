#ifndef NO_ALGORITHM_H
#define NO_ALGORITHM_H

#include "inttypes.h"
#include "data_structures.h"
#include "TASK_MPPT/task_mppt.h"

enum Algorithm_Status {ALGORITHM_NOT_DONE, ALGORITHM_DONE};

void No_algorithm(MPPTData *MPPTData_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status);

#endif // NO_ALGORITHM_H