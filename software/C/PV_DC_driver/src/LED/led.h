#ifndef H_LED
#define H_LED

#include <inttypes.h>

enum LED_state {LED_ON, LED_OFF};

void Led_Init(uint8_t green_pin, uint8_t red_pin);
void Led_green_set(enum LED_state eLed_state);
void Led_red_set(enum LED_state eLed_state);

#endif //H_LED