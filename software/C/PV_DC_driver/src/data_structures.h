#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <inttypes.h>

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

#endif // DATA_STRUCTURES_H