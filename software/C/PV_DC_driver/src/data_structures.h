#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <inttypes.h>

//========================QUEUES==========================

typedef struct {
    uint64_t second_number;
    uint64_t energy_j;
} TimerData;

typedef struct {
    uint8_t desired_temperature;
    uint16_t boiler_capacity;
} BoilerSettings;

typedef struct {
    double voltage_value;
    double current_value;
    double power_value;
} ElectricalMeasurements;

typedef struct {
    float temp_water;
    float temp_case; 
} TemperatureReadings;

//========================TASKS===========================

typedef struct {
    QueueHandle_t BoilerSettings_queue;
    QueueHandle_t ElectricalMeasurements_queue;
    QueueHandle_t TemperatureReadings_queue;
    QueueHandle_t TimerData_queue;
} TaskUserParameters;

#endif // DATA_STRUCTURES_H