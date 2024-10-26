#ifndef H_BUTTONS
#define H_BUTTONS

#include <inttypes.h>

enum Button_Name {BUTTON_0, BUTTON_1, BUTTON_2};
enum Button_State {PRESSED, RELEASED};

void Button_Init(uint8_t pin0, uint8_t pin1, uint8_t pin2);
enum Button_State eButton_Read(enum Button_Name eButton_Name);

#endif // H_BUTTONS