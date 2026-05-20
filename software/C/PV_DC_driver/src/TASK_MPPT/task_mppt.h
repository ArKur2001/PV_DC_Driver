#ifndef TASK_MPPT_H
#define TASK_MPPT_H

enum Task_MPPT_state    {RECEIVE, MEASUREMENTS, MPPT, SEND};

void Task_MPPT(void *pvParameters);

#endif // TASK_MPPT_H