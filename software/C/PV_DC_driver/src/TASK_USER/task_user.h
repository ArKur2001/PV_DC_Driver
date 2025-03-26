#ifndef TASK_USER_H
#define TASK_USER_H

void Task_User(QueueHandle_t BoilerSettings_queue, QueueHandle_t ElectricalMeasurements_queue, QueueHandle_t TemperatureReadings_queue, uint64_t second_number, uint64_t energy_j);

#endif // TASK_USER_H