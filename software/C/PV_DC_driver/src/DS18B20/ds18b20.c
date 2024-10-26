#include "DS18B20/ds18b20.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "rom/ets_sys.h"

int ds18b20_reset(uint8_t pin) 
{
    int response = 0;
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    ets_delay_us(480);
    gpio_set_level(pin, 1);
    ets_delay_us(70);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    response = gpio_get_level(pin);
    ets_delay_us(410);
    
    if(response == 0)
    {
        return 1;
    }
    else
    {
        return -1;
    } 
}

void ds18b20_write_bit(uint8_t pin, int bit) 
{
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);

    if(bit == true)
    {
        ets_delay_us(10);
    }
    else
    {
        ets_delay_us(65);
    }

    gpio_set_level(pin, 1);

    if(bit == true)
    {
        ets_delay_us(55);
    }
    else
    {
        ets_delay_us(5);
    }
}

int ds18b20_read_bit(uint8_t pin) 
{
    int bit = 0;
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    ets_delay_us(3);
    gpio_set_level(pin, 1);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    ets_delay_us(10);
    bit = gpio_get_level(pin);
    ets_delay_us(45);

    return bit;
}

void ds18b20_write_byte(uint8_t pin, int byte) 
{
    for (uint8_t i = 0; i < 8; i++) 
    {
        ds18b20_write_bit(pin, byte & 0x01);
        byte >>= 1;
    }
}

int ds18b20_read_byte(uint8_t pin) 
{
    int byte = 0;
    for (uint8_t i = 0; i < 8; i++) 
    {
        byte |= (ds18b20_read_bit(pin) << i);
    }

    return byte;
}

float ds18b20_get_temp(uint8_t pin) 
{
    if (ds18b20_reset(pin) == 1) {
        ds18b20_write_byte(pin, 0xCC); // SKIP ROM
        ds18b20_write_byte(pin, 0x44); // CONVERT T
        vTaskDelay(pdMS_TO_TICKS(375)); // Czas konwersji dla 11 bitów

        ds18b20_reset(pin);
        ds18b20_write_byte(pin, 0xCC); // SKIP ROM
        ds18b20_write_byte(pin, 0xBE); // READ SCRATCHPAD

        int temp_LSB = ds18b20_read_byte(pin);
        int temp_MSB = ds18b20_read_byte(pin);
        int16_t temp = (temp_MSB << 8) | temp_LSB;

        return temp / 16.0; // Celcius degree conversion
    } 
    else 
    {
        return -10.0; // Error code
    }
}
