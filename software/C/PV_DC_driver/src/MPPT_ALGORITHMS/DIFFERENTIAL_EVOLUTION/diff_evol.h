#ifndef DIFF_EVOL_H
#define DIFF_EVOL_H

#include "inttypes.h"
#include "data_structures.h"
#include "TASK_MPPT/task_mppt.h"

enum Algorithm_Status {ALGORITHM_NOT_DONE, ALGORITHM_DONE};

void Differential_Evolution_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status);

#endif // DIFF_EVOL_H