//
// Created by Luke Nel on 11/05/2023.
//

#ifndef PID_CONTROLLER_LCD_H
#define PID_CONTROLLER_LCD_H

void LCD_Init(void);
void LCD_Print(const char *msg);
void LCD_Print_int(int val);
void LCD_Print_float(float val);
void LCD_Cursor(char row, char pos);
void LCD_Clear();

#endif //PID_CONTROLLER_LCD_H
