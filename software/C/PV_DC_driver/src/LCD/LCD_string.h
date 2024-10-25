#ifndef H_LCD_STRING
#define H_LCD_STRING

#include <inttypes.h>

void LCD_INFO_state();
void LCD_TEMPERATURE_state();
void LCD_BOILER_CAPACITY_state();
void LCD_BOILER_CAPACITY_setting(uint16_t capacity);
void LCD_TEMPERATURE_setting(uint16_t temp);
void LCD_INFO_power(double power);
void LCD_INFO_temp(double temp);
void LCD_INFO_voltage(double voltage);
void LCD_INFO_current(double current);
void LCD_INFO_energy(double energy_j);
void LCD_INFO_hours(uint8_t hour);
void LCD_INFO_minutes(uint8_t minute);

#endif //H_LCD_STRING 