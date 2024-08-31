#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h"

void app_main() 
{

    gpio_set_direction(2, GPIO_MODE_OUTPUT);

    while(1)
    {
        gpio_set_level(2, 1);
        printf("led_on\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        
        gpio_set_level(2, 0);
        printf("led_off\n");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}