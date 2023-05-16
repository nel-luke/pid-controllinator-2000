//
// Created by Luke Nel on 11/05/2023.
//

#ifndef PID_CONTROLLER_LCD_H
#define PID_CONTROLLER_LCD_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void lcd_init(I2C_HandleTypeDef* handle, uint8_t addr);
void lcd_start_payload();
void lcd_stop_payload();
void lcd_send_payload();

void lcd_clear_display(void);
void lcd_set_on(bool display, bool cursor, bool blinking);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print_c(char c);
void lcd_print(const char* str);
void lcd_print_int(int val);
void lcd_print_float(float val);

#endif //PID_CONTROLLER_LCD_H
