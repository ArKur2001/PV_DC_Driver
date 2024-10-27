#ifndef H_MEMORY
#define H_MEMORY

#include <inttypes.h>

void flash_read(uint8_t *desired_water_temp, uint16_t *boiler_cap, uint64_t *produced_energy);
void flash_write(uint8_t desired_water_temp, uint16_t boiler_cap, uint64_t produced_energy);

#endif //MEMORY