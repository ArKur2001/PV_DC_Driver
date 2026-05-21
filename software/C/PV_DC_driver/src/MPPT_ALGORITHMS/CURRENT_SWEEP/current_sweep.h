#ifndef _CURRENT_SWEEP_H
#define _CURRENT_SWEEP_H

#include "inttypes.h"
#include "data_structures.h"
#include "TASK_MPPT/task_mppt.h"

enum Algorithm_Status {ALGORITHM_NOT_DONE, ALGORITHM_DONE};

void Current_Sweep_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status);

#endif // _CURRENT_SWEEP_H