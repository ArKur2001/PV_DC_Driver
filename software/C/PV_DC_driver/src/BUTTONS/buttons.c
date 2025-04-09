#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include <BUTTONS/buttons.h>
#include <esp_log.h>

#define BUTTON_STATE_CHECK_NUMBER 2

const char* TAG = "BUTTONS";

enum Button_Name eButton_Name;

enum Button_State ebut0_level = RELEASED; 
enum Button_State ebut1_level = RELEASED; 
enum Button_State ebut2_level = RELEASED; 

uint8_t but0_pin = 0;
uint8_t but1_pin = 0;
uint8_t but2_pin = 0;

void Set_Button_0_state(void *arg)
{
    uint8_t counter = 0;

    gpio_intr_disable(but0_pin);

    ets_delay_us(10);

    for (uint8_t i = 0 ; i < BUTTON_STATE_CHECK_NUMBER; i++)
    {
        if(gpio_get_level(but0_pin) == 0)
        {
            counter++;
        }
        else
        {
            counter = counter;
        }

        ets_delay_us(10);
    }


    if(counter == BUTTON_STATE_CHECK_NUMBER)
    {
        ebut0_level = PRESSED;
    }
    else
    {
        ebut0_level = RELEASED;

        gpio_intr_enable(but0_pin);
    } 
}

void Set_Button_1_state(void* arg)
{   
    uint8_t counter = 0;

    gpio_intr_disable(but1_pin);

    ets_delay_us(10);

    for (uint8_t i = 0 ; i < BUTTON_STATE_CHECK_NUMBER ; i++)
    {
        if(gpio_get_level(but1_pin) == 0)
        {
            counter++;
        }
        else
        {
            counter = counter;
        }

        ets_delay_us(10);
    }
    

    if(counter == BUTTON_STATE_CHECK_NUMBER)
    {
        ebut1_level = PRESSED;
    }
    else
    {
        ebut1_level = RELEASED;

        gpio_intr_enable(but1_pin);
    } 
} 

void Set_Button_2_state(void* arg)
{
    uint8_t counter = 0;

    gpio_intr_disable(but2_pin);

    ets_delay_us(10);

    for (uint8_t i = 0 ; i < BUTTON_STATE_CHECK_NUMBER ; i++)
    {
        if(gpio_get_level(but2_pin) == 0)
        {
            counter++;
        }
        else
        {
            counter = counter;
        }

        ets_delay_us(10);
    }


    if(counter == BUTTON_STATE_CHECK_NUMBER)
    {
        ebut2_level = PRESSED;
    }
    else
    {
        ebut2_level = RELEASED;

        gpio_intr_enable(but2_pin);
    } 
} 

void Button_Init(uint8_t pin0, uint8_t pin1, uint8_t pin2)
{
    but0_pin = pin0;
    but1_pin = pin1;
    but2_pin = pin2;

    ESP_ERROR_CHECK(gpio_set_direction(but0_pin, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(but1_pin, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(but2_pin, GPIO_MODE_INPUT));

    ESP_ERROR_CHECK(gpio_pullup_en(but0_pin));
    ESP_ERROR_CHECK(gpio_pullup_en(but1_pin));
    ESP_ERROR_CHECK(gpio_pullup_en(but2_pin));
   
    gpio_install_isr_service(0);

    gpio_set_intr_type(but0_pin, GPIO_INTR_LOW_LEVEL);
    gpio_set_intr_type(but1_pin, GPIO_INTR_LOW_LEVEL);
    gpio_set_intr_type(but2_pin, GPIO_INTR_LOW_LEVEL);

    gpio_isr_handler_add(but0_pin, Set_Button_0_state, NULL);
    gpio_isr_handler_add(but1_pin, Set_Button_1_state, NULL);
    gpio_isr_handler_add(but2_pin, Set_Button_2_state, NULL);
}

enum Button_State eButton_Read(enum Button_Name eButton_Name)
{
    enum Button_State ebut_level = RELEASED;

    switch(eButton_Name)
    {
        case BUTTON_0:
            if(gpio_get_level(but0_pin) == 1)
            {
                ebut_level = ebut0_level;
                ebut0_level = RELEASED; 

                gpio_intr_enable(but0_pin);

                return ebut_level;
            }
            else
            {
                return RELEASED;
            }
            break;

        case BUTTON_1:
            if(gpio_get_level(but1_pin) == 1)
            {
                ebut_level = ebut1_level;
                ebut1_level = RELEASED; 
            
                gpio_intr_enable(but1_pin);
            
                return ebut_level;
            }
            else
            {
                return RELEASED;
            }
            break;

        case BUTTON_2:
            if(gpio_get_level(but2_pin) == 1)
            {
                ebut_level = ebut2_level;
                ebut2_level = RELEASED; 

                gpio_intr_enable(but2_pin);

                return ebut_level;
            }
            else
            {
                return RELEASED;
            }
            break;

        default:
            return RELEASED;
            break;
    }
}
