#include "driver/gpio.h"
#include "LED/led.h"


uint8_t green_led_pin = 0;            
uint8_t red_led_pin = 0;

void Led_Init(uint8_t green_pin, uint8_t red_pin)
{
    green_led_pin = green_pin;
    red_led_pin = red_pin;

    gpio_set_direction(green_led_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(red_led_pin, GPIO_MODE_OUTPUT);
}

void Led_green_set(enum LED_state eLed_state)
{
    switch (eLed_state)
    {
        case LED_ON:
            gpio_set_level(green_led_pin, 1);
        break;

        case LED_OFF:
            gpio_set_level(green_led_pin, 0);
        break;
   
        default:
            gpio_set_level(green_led_pin, 0);
        break;
   }
}

void Led_red_set(enum LED_state eLed_state)
{
    switch (eLed_state)
    {
        case LED_ON:
            gpio_set_level(red_led_pin, 1);
        break;

        case LED_OFF:
            gpio_set_level(red_led_pin, 0);
        break;
   
        default:
            gpio_set_level(red_led_pin, 0);
        break;
   }
}