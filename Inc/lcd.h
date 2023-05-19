//
// Created by Luke Nel on 11/05/2023.
//

#ifndef PID_CONTROLLER_LCD_H
#define PID_CONTROLLER_LCD_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* I2C address */
#ifndef LCD_I2C_ADDR
#define LCD_I2C_ADDR         (0x27<<1)
#endif

HAL_StatusTypeDef LCD_Init(I2C_HandleTypeDef* handle);
void LCD_Begin_Payload(void);
void LCD_End_Payload(void);
void LCD_Send_Payload(void);

void LCD_Clear_Display(void);
void LCD_Enable(bool display, bool cursor, bool blinking);
void LCD_Set_Cursor(uint8_t row, uint8_t col);
void LCD_Print_c(char c);
void LCD_Print_s(const char* str);
void LCD_Print_i(int val);
void LCD_Print_f(float val);
void LCD_Print_ff(float val, size_t width, size_t precision);

#endif //PID_CONTROLLER_LCD_H
