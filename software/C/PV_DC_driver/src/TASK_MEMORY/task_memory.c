#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "TASK_MEMORY/task_memory.h"
#include "MEMORY/memory.h"
#include "data_structures.h"

#define MEMORY_WRITE_PERIOD     3600    //s

enum Task_Memory_state  {RECEIVE, WAIT, WRITE_MEMORY};

void WriteMemory(BoilerSettings BoilerSettings_data, TimerData TimerData_data)
{
    uint8_t desired_temperature_tmp;
    uint16_t boiler_capacity_tmp;
    uint64_t energy_j_tmp;

    flash_read(&desired_temperature_tmp, &boiler_capacity_tmp, &energy_j_tmp);

    if(desired_temperature_tmp != BoilerSettings_data.desired_temperature || boiler_capacity_tmp != BoilerSettings_data.boiler_capacity || energy_j_tmp != TimerData_data.energy_j)
    {
        flash_write(BoilerSettings_data.desired_temperature, BoilerSettings_data.boiler_capacity, TimerData_data.energy_j);
    }
}

void Task_Memory(void *pvParameters)
{
    TaskMemoryParameters *params = (TaskMemoryParameters *)pvParameters;

    QueueHandle_t BoilerSettings_queue = params->BoilerSettings_queue;
    QueueHandle_t TimerData_queue = params->TimerData_queue;

    static enum Task_Memory_state eTask_Memory_state = WAIT;

    static BoilerSettings BoilerSettings_data = {50, 100};
    static TimerData TimerData_data = {0, 0};

    static uint64_t second_number_mem = MEMORY_WRITE_PERIOD;

    while(1)
    {
        switch (eTask_Memory_state)
        {
            case RECEIVE:
                if (xQueuePeek(BoilerSettings_queue, &BoilerSettings_data, pdMS_TO_TICKS(0)) == pdTRUE && 
                    xQueuePeek(TimerData_queue, &TimerData_data, pdMS_TO_TICKS(0)) == pdTRUE)
                {
                    eTask_Memory_state = WAIT;
                }
                else
                {
                    eTask_Memory_state = RECEIVE;

                    vTaskDelay(pdMS_TO_TICKS(1000));
                }

                break;

            case WAIT:
                if(TimerData_data.second_number > second_number_mem)
                {
                    second_number_mem = TimerData_data.second_number + MEMORY_WRITE_PERIOD;
        
                    eTask_Memory_state = WRITE_MEMORY;
                }
                else
                {
                    eTask_Memory_state = RECEIVE;
                }

                vTaskDelay(pdMS_TO_TICKS(10));

                break;

            case WRITE_MEMORY:
                WriteMemory(BoilerSettings_data, TimerData_data);

                eTask_Memory_state = RECEIVE;

                vTaskDelay(pdMS_TO_TICKS(1000));

            break;

            default:
                eTask_Memory_state = RECEIVE;

                vTaskDelay(pdMS_TO_TICKS(1000));

                break;
        }
    }
}