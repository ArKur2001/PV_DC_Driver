#include "driver/gpio.h"
#include <BUTTONS/buttons.h>

#include <esp_log.h>

const char* TAG = "BUTTONS";

enum Button_Name eButton_Name;

enum Button_State ebut0_level = RELEASED; 
enum Button_State ebut1_level = RELEASED; 


void Set_Button_0_state(void *arg)
{
    ebut0_level = PRESSED;
}

void Set_Button_1_state(void* arg)
{
    ebut1_level = PRESSED;
} 

void Button_Init(uint8_t pin0, uint8_t pin1)
{
    ESP_ERROR_CHECK(gpio_set_direction(pin0, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(pin1, GPIO_MODE_INPUT));

    ESP_ERROR_CHECK(gpio_pullup_en(pin0));
    ESP_ERROR_CHECK(gpio_pullup_en(pin1));
   
    gpio_install_isr_service(0);

    gpio_set_intr_type(pin0, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(pin1, GPIO_INTR_POSEDGE);

    gpio_isr_handler_add(pin0, Set_Button_0_state, NULL);
    gpio_isr_handler_add(pin1, Set_Button_1_state, NULL);
}

enum Button_State eButton_Read(enum Button_Name eButton_Name)
{
    enum Button_State ebut_level = RELEASED;

    switch(eButton_Name)
    {
    case BUTTON_0:
        ebut_level = ebut0_level;
        ebut0_level = RELEASED; 
        return ebut_level;
        break;
    case BUTTON_1:
        ebut_level = ebut1_level;
        ebut1_level = RELEASED; 
        return ebut_level;
        break;
    default:
        return RELEASED;
        break;
    }
}
