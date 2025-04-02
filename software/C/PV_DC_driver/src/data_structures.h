#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <inttypes.h>
#include "freertos/queue.h"

enum MPPT_Permission {MPPT_NOT_ALLOWED, MPPT_ALLOWED};

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

typedef struct {
    double power_opt;
    enum MPPT_Permission eMPPT_Permission;
} MPPTData;


//========================TASKS===========================

typedef struct {
    QueueHandle_t BoilerSettings_queue;
    QueueHandle_t ElectricalMeasurements_queue;
    QueueHandle_t TemperatureReadings_queue;
    QueueHandle_t TimerData_queue;
} TaskUserParameters;

typedef struct {
    QueueHandle_t BoilerSettings_queue;
    QueueHandle_t ElectricalMeasurements_queue;
    QueueHandle_t TemperatureReadings_queue;
    QueueHandle_t TimerData_queue;
    QueueHandle_t MPPTData_queue;
    uint8_t boiler_sensor_pin;
    uint8_t case_sensor_pin;
} TaskControlParameters;

typedef struct {
    QueueHandle_t BoilerSettings_queue;
    QueueHandle_t TimerData_queue;
} TaskMemoryParameters;

typedef struct {
    QueueHandle_t ElectricalMeasurements_queue;
    QueueHandle_t MPPTData_queue;
    u_int8_t adc_voltage_pin; 
    u_int8_t adc_current_pin;
    u_int8_t pwm_duty_resolution_bit;
} TaskMPPTParameters;

#endif // DATA_STRUCTURES_H