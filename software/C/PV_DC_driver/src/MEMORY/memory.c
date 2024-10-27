#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "nvs_flash.h"
#include "MEMORY/memory.h"

void flash_read(uint8_t *desired_water_temp, uint16_t *boiler_cap, uint64_t *produced_energy)
{
    nvs_flash_init();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    nvs_handle_t program_data;

    nvs_open("storage", NVS_READONLY, &program_data);
    nvs_get_u8(program_data, "desired_temp", desired_water_temp);
    nvs_get_u16(program_data, "boiler_cap", boiler_cap);
    nvs_get_u64(program_data, "produced_energy", produced_energy);

    nvs_close(program_data);

    nvs_flash_deinit();
}

void flash_write(uint8_t desired_water_temp, uint16_t boiler_cap, uint64_t produced_energy)
{
    nvs_flash_init();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    nvs_handle_t program_data;

    nvs_open("storage", NVS_READWRITE, &program_data);
    nvs_set_u8(program_data, "desired_temp", desired_water_temp);
    nvs_set_u16(program_data, "boiler_cap", boiler_cap);
    nvs_set_u64(program_data, "produced_energy", produced_energy);

    nvs_commit(program_data);
    nvs_close(program_data);

    nvs_flash_deinit();
}