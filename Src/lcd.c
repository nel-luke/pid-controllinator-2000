//
// Created by Luke Nel on 11/05/2023.
//

#include "lcd.h"

#include <stdio.h>
#include <malloc.h>
#include <memory.h>

static I2C_HandleTypeDef* i2c_handle = NULL;

#define CMD_CLEAR_DISP      0x01
#define CMD_RET_HOME        0x02
#define CMD_DDRAM_DEC       0x04
#define CMD_DISP_DEC        0x05
#define CMD_DDRAM_INC       0x06
#define CMD_DISP_INC        0x07
#define CMD_DCB_OFF         0x08
#define CMD_CUR_SHL         0x10
#define CMD_CUR_SHR         0x14
#define CMD_DISP_SHL        0x18
#define CMD_DISP_SHR        0x1C
#define CMD_DNF_SET         0x28
#define CMD_DNF_UNSET       0x20
#define CMD_SET_CGRAM       0x40
#define CMD_SET_DDRAM       0x80
#define CMD_WAIT            0x0E0E0A0A // Do not use with CMD()!

#define R_U(a)              ((a) & 0xF0)
#define R_L(a)              ((a << 4) & 0xF0)

#define CMD_U(a)            (((R_L(a)|0x08)<<24) | ((R_L(a)|0x0C)<<16))
#define CMD_L(a)            (((R_U(a)|0x08)<< 8) | ((R_U(a)|0x0C)<< 0))
#define CMD(a)              (CMD_U(a) | CMD_L(a))

#define DATA_U(a)            (((R_L(a)|0x09)<<24) | ((R_L(a)|0x0D)<<16))
#define DATA_L(a)            (((R_U(a)|0x09)<< 8) | ((R_U(a)|0x0D)<< 0))
#define LCD_DATA(a)              (DATA_U(a) | DATA_L(a))

static const uint32_t lcd_delay[] = { CMD_WAIT, CMD_WAIT, CMD_WAIT };

struct lcd_cmd_type_t {
    uint32_t cmd;
    struct lcd_cmd_type_t* next;
};
typedef struct lcd_cmd_type_t lcd_cmd;

static lcd_cmd* cmd_list = NULL;
static uint8_t* cmd_buff = NULL;
static size_t cmd_buff_length = 0;


static inline void new_cmd(uint32_t cmd) {
    lcd_cmd* tmp = calloc(1, sizeof(lcd_cmd));
    tmp->cmd = cmd;
    tmp->next = cmd_list;
    cmd_list = tmp;
}

#define MAKE_FUNC(a, b) \
void a(void) { \
    new_cmd(CMD(b)); \
}

void LCD_Clear_Display(void) {
    new_cmd(CMD(CMD_CLEAR_DISP));
    new_cmd(CMD(CMD_CLEAR_DISP));
}

static MAKE_FUNC(lcd_Set_ddram_decrement, CMD_DDRAM_DEC)
static MAKE_FUNC(lcd_shift_display_right, CMD_DISP_DEC)
static MAKE_FUNC(lcd_set_ddram_increment, CMD_DDRAM_INC)
static MAKE_FUNC(lcd_shift_display_left, CMD_DISP_INC)

void LCD_Enable(bool display, bool cursor, bool blinking) {
    new_cmd(CMD(CMD_DCB_OFF));

    if (display)
        cmd_list->cmd |= 0x40400000;

    if (cursor)
        cmd_list->cmd |= 0x20200000;

    if (blinking)
        cmd_list->cmd |= 0x10100000;
}

static MAKE_FUNC(lcd_shift_cursor_left, CMD_CUR_SHL)
static MAKE_FUNC(lcd_shift_cursor_right, CMD_CUR_SHR)
static MAKE_FUNC(lcd_shift_dispcur_left, CMD_DISP_SHL)
static MAKE_FUNC(lcd_shift_dispcur_right, CMD_DISP_SHR)
static MAKE_FUNC(lcd_dnf_set, CMD_DNF_SET)
static MAKE_FUNC(lcd_dnf_unset, CMD_DNF_UNSET)

void LCD_Set_Cursor(uint8_t row, uint8_t col) {
    new_cmd(0);

    cmd_list->cmd = row == 0 ? CMD((0x80 | (col&0x0F)))
            : CMD((0xC0 | (col&0x0F)));
}

void LCD_Print_c(char c) {
    new_cmd(LCD_DATA(c));
}

void LCD_Print_s(const char* str) {
    for (char const* i = str; *i != '\0'; i++) {
        new_cmd(LCD_DATA(*i));
    }
}

void LCD_Print_i(int val) {
    char integer[16] = { 0 };
    sprintf(integer, "%d", val);
    LCD_Print_s(integer);
}

void LCD_Print_f(float val) {
    char integer[16] = { 0 };
    sprintf(integer, "%f", val);
    LCD_Print_s(integer);
}

void LCD_Print_ff(float val, size_t width, size_t precision) {
    char integer[16] = { 0 };
    sprintf(integer, "%0*.*f", width, precision, val);
    LCD_Print_s(integer);
}

void LCD_Begin_Payload() {
    cmd_list = calloc(1, sizeof(lcd_cmd));
}

void LCD_End_Payload() {
    lcd_cmd* i = cmd_list;
    cmd_buff_length = 0;
    while(i->next != NULL) {
        cmd_buff_length += 4;
        i = i->next;
    }

    while (HAL_I2C_GetState(i2c_handle) != HAL_I2C_STATE_READY) {
        HAL_Delay(1);
    }

    free(cmd_buff);
    cmd_buff = calloc(cmd_buff_length, sizeof(uint32_t));

    size_t j = (cmd_buff_length-4)*4;
    while (cmd_list->next != NULL) {
        memcpy(cmd_buff + j, &cmd_list->cmd, 4);
        memcpy(cmd_buff + j+4, lcd_delay, 12);
        j -= 16;
        lcd_cmd* tmp = cmd_list->next;
        free(cmd_list);
        cmd_list = tmp;
    }
    free(cmd_list);
}

//void lcd_get_payload_length(size_t* length) {
//    *length = cmd_buff_length;
//}
//
//void lcd_get_payload(uint32_t* payload) {
//    memcpy(payload, cmd_buff, cmd_buff_length * sizeof(uint32_t));
//}
//
//void lcd_load_payload(const uint32_t* payload, size_t length) {
//    memcpy(cmd_buff, payload, length * sizeof(uint32_t));
//}

void LCD_Send_Payload() {
    HAL_I2C_Master_Transmit_DMA(i2c_handle, LCD_I2C_ADDR, (uint8_t*)cmd_buff, sizeof(uint32_t)*cmd_buff_length);
}

HAL_StatusTypeDef LCD_Init(I2C_HandleTypeDef* handle) {
    i2c_handle = handle;

    /* Check if LCD connected to I2C */
    if (HAL_I2C_IsDeviceReady(i2c_handle, LCD_I2C_ADDR, 1, 20000) != HAL_OK) {
        /* Return false */
        return HAL_ERROR;
    }

    LCD_Begin_Payload();
    lcd_dnf_unset();
    lcd_dnf_set();
    LCD_Enable(false, false, false);
    LCD_Clear_Display();
    lcd_set_ddram_increment();
    LCD_Enable(true, false, false);
    LCD_End_Payload();

    HAL_Delay(100);
    LCD_Send_Payload();

    return HAL_OK;
}