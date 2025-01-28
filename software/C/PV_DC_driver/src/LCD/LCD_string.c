#include "LCD/LCD_string.h"
#include "LCD/HD44780.h"
#include <stdio.h>

void build_string(char *buffer, int width, double number, const char *unit, int precision)
{
    char format[10];
    snprintf(format, sizeof(format), "%%.%df%%s", precision);

    char temporary[20];
    snprintf(temporary, sizeof(temporary), format, number, unit);

    snprintf(buffer, width, "%-*s", width, temporary); 
}

void LCD_BOILER_CAPACITY_state()
{
    LCD_clearScreen();
    LCD_setCursor(0, 0);
    LCD_writeStr("  Boiler capacity:  ");
}

void LCD_BOILER_CAPACITY_setting(uint16_t capacity)
{
    char str_buf[5];

    build_string(str_buf, 7, capacity, " L", 0);
    LCD_setCursor(8, 2);
    LCD_writeStr(str_buf);
}

void LCD_TEMPERATURE_state()
{
    LCD_clearScreen();
    LCD_setCursor(0, 0);
    LCD_writeStr("Temperature setting:");
}

void LCD_TEMPERATURE_setting(uint16_t temp)
{
    char str_buf[7];
    char symbol[5];
    symbol[0] = ' ';
    symbol[1] = 223;
    symbol[2] = 'C';
    symbol[3] = ' ';
    symbol[4] = '\0';
    
    build_string(str_buf, 9, temp, symbol, 0);
    LCD_setCursor(8, 2);
    LCD_writeStr(str_buf);
}

void LCD_INFO_state()
{
    LCD_clearScreen();
    LCD_setCursor(0, 0);
    LCD_writeStr("P:        T:        ");
    LCD_setCursor(0, 1);
    LCD_writeStr("U:        I:        ");
    LCD_setCursor(0, 2);
    LCD_writeStr("Etot:               ");
    LCD_setCursor(0, 3);
    LCD_writeStr("Heating end:        ");
}

void LCD_INFO_power(double power)
{
    char str_buf[7];

    build_string(str_buf, 7, power, " W", 0);
    LCD_setCursor(3, 0);
    LCD_writeStr(str_buf);
}

void LCD_INFO_temp(double temp)
{
    char str_buf[7];
    char symbol[3];
    symbol[0] = ' ';
    symbol[1] = 223;
    symbol[2] = 'C';
    
    build_string(str_buf, 8, temp, symbol, 1);
    LCD_setCursor(13, 0);

    if (temp <= 0)
    {
        LCD_writeStr("ERROR  ");
    }
    else
    {
        LCD_writeStr(str_buf);
    }
}

void LCD_INFO_voltage(double voltage)
{
    char str_buf[7];
    
    build_string(str_buf, 6, voltage, " V", 0);
    LCD_setCursor(3, 1);
    LCD_writeStr(str_buf);
}

void LCD_INFO_current(double current)
{
    char str_buf[7];
    
    build_string(str_buf, 7, current, " A", 1);
    LCD_setCursor(13, 1);
    LCD_writeStr(str_buf);
}

void LCD_INFO_energy(double energy_j)
{
    char str_buf[14];
    double energy_kwh = energy_j / 3600000.0;

    build_string(str_buf, 14, energy_kwh, " kWh", 1);
    LCD_setCursor(6, 2);
    LCD_writeStr(str_buf);
}

void LCD_INFO_hours(uint8_t hour)
{
    char str_buf[5];
    
    build_string(str_buf, 5, hour, "h ", 0);
    LCD_setCursor(13, 3);
    LCD_writeStr(str_buf);
}

void LCD_INFO_minutes(uint8_t minute)
{
    char str_buf[4];

    build_string(str_buf, 4, minute, "m", 0);
    LCD_setCursor(17, 3);
    LCD_writeStr(str_buf);
}

void LCD_INFO_time(uint8_t hour, uint8_t minute)
{
    if(hour >= 24)
    {
        LCD_setCursor(13, 3);
        LCD_writeStr("> 24h  ");
    }
    else
    {
        LCD_INFO_hours(hour);
        LCD_INFO_minutes(minute);
    }   
}