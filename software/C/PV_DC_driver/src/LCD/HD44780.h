#ifndef H_HD44780
#define H_HD44780

enum Backligt_state {OFF, ON};

void set_backlight_state(enum Backligt_state eBacklight_state);

void LCD_init(uint8_t addr, uint8_t dataPin, uint8_t clockPin, uint8_t cols, uint8_t rows);
void LCD_setCursor(uint8_t col, uint8_t row);
void LCD_home(void);
void LCD_clearScreen(void);
void LCD_writeChar(char c);
void LCD_writeStr(char* str); 

#endif //H_HD44780 