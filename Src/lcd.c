//
// Created by Luke Nel on 11/05/2023.
//

#include "lcd.h"

#include <stdio.h>
#include "stm32f4xx_hal.h"

#define SLAVE_ADDRESS_LCD 0x27

extern I2C_HandleTypeDef hi2c1;
static I2C_HandleTypeDef* i2c_handle = NULL;
static uint8_t lcd_addr = 0;

static void lcd_send_cmd(char cmd) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C;  //en=1, rs=0
    data_t[1] = data_u | 0x08;  //en=0, rs=0
    data_t[2] = data_l | 0x0C;  //en=1, rs=0
    data_t[3] = data_l | 0x08;  //en=0, rs=0
    HAL_I2C_Master_Transmit(i2c_handle, SLAVE_ADDRESS_LCD << 1, (uint8_t *) data_t, 4, 100);
}

static void lcd_send_data(char data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D;  //en=1, rs=1
    data_t[1] = data_u | 0x09;  //en=0, rs=1
    data_t[2] = data_l | 0x0D;  //en=1, rs=1
    data_t[3] = data_l | 0x09;  //en=0, rs=1
    HAL_I2C_Master_Transmit(i2c_handle, SLAVE_ADDRESS_LCD << 1, (uint8_t *) data_t, 4, 100);
}

void LCD_Init(void) {
    i2c_handle = &hi2c1;
    lcd_addr = SLAVE_ADDRESS_LCD;

    // 4 bit initialisation
    HAL_Delay(50);  // wait for >40ms
    lcd_send_cmd(0x30);
    HAL_Delay(5);  // wait for >4.1ms
    lcd_send_cmd(0x30);
    HAL_Delay(1);  // wait for >100us
    lcd_send_cmd(0x30);
    HAL_Delay(10);
    lcd_send_cmd(0x20);  // 4bit mode
    HAL_Delay(10);

    // dislay initialisation
    lcd_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    HAL_Delay(1);
    lcd_send_cmd(0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
    HAL_Delay(1);
    lcd_send_cmd(0x01);  // clear display
    HAL_Delay(1);
    HAL_Delay(1);
    lcd_send_cmd(0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    HAL_Delay(1);
    lcd_send_cmd(0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)

    HAL_Delay(100);
}

void LCD_Print(const char *msg) {
    while ((*msg) != 0) {
        lcd_send_data(*msg);
        msg++;
    }
}

void LCD_Print_int(int val) {
    char LCD_BUF[20] = {0};

    sprintf(LCD_BUF, "%d", val);

    const char *msg = &LCD_BUF[0];

    while ((*msg) != 0) {
        lcd_send_data(*msg);
        msg++;
    }
}

void LCD_Print_float(float val) {
    char LCD_BUF[20] = {0};

    snprintf(LCD_BUF, 20, "%.1f", val);

    LCD_Print(LCD_BUF);
}

void LCD_Cursor(char row, char pos) {
    unsigned char location = 0;
    if (row == 0) {
        location = (0x80) | ((pos) & 0x0f);
        lcd_send_cmd(location);
    } else {
        location = (0xC0) | ((pos) & 0x0f);
        lcd_send_cmd(location);
    }
}

void LCD_Clear() {
    lcd_send_cmd(0x01);
    HAL_Delay(3);
}