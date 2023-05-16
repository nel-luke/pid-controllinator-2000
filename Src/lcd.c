//
// Created by Luke Nel on 11/05/2023.
//

#include "lcd.h"

#include <stdio.h>
#include <malloc.h>
#include <memory.h>

static I2C_HandleTypeDef* i2c_handle = NULL;
static uint8_t lcd_addr = 0;

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

#define CLEAR_DISP          0
#define DDRAM_DEC           1
#define DISP_DEC            2
#define DDRAM_INC           3
#define DISP_INC            4
#define DCB_OFF             5
#define CURSOR_LEFT         6
#define CURSOR_RIGHT        7
#define DISPLAY_LEFT        8
#define DISPLAY_RIGHT       9
#define DNF_SET             10
#define DNF_UNSET           11
#define SET_CGRAM           12
#define SET_DDRAM           13
#define TOTAL_CMDS          14

static const uint32_t lcd_cmds[TOTAL_CMDS] = {
    CMD(CMD_CLEAR_DISP),
    CMD(CMD_DDRAM_DEC),
    CMD(CMD_DISP_DEC),
    CMD(CMD_DDRAM_INC),
    CMD(CMD_DISP_INC),
    CMD(CMD_DCB_OFF),
    CMD(CMD_CUR_SHL),
    CMD(CMD_CUR_SHR),
    CMD(CMD_DISP_SHL),
    CMD(CMD_DISP_SHR),
    CMD(CMD_DNF_SET),
    CMD(CMD_DNF_UNSET),
    CMD(CMD_SET_CGRAM),
    CMD(CMD_SET_DDRAM),
};
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
inline void a(void) { \
    new_cmd(lcd_cmds[b]); \
}

inline void lcd_clear_display(void) {
    new_cmd(lcd_cmds[CLEAR_DISP]);
    new_cmd(lcd_cmds[CLEAR_DISP]);
}

MAKE_FUNC(lcd_set_ddram_decrement, DDRAM_DEC)
MAKE_FUNC(lcd_shift_display_right, DISP_DEC)
MAKE_FUNC(lcd_set_ddram_increment, DDRAM_INC)
MAKE_FUNC(lcd_shift_display_left, DISP_INC)

void lcd_set_on(bool display, bool cursor, bool blinking) {
    new_cmd(lcd_cmds[DCB_OFF]);

    if (display)
        cmd_list->cmd |= 0x40400000;

    if (cursor)
        cmd_list->cmd |= 0x20200000;

    if (blinking)
        cmd_list->cmd |= 0x10100000;
}

MAKE_FUNC(lcd_shift_cursor_left, CURSOR_LEFT)
MAKE_FUNC(lcd_shift_cursor_right, CURSOR_RIGHT)
MAKE_FUNC(lcd_shift_dispcur_left, DISPLAY_LEFT)
MAKE_FUNC(lcd_shift_dispcur_right, DISPLAY_RIGHT)
static MAKE_FUNC(lcd_dnf_set, DNF_SET)
static MAKE_FUNC(lcd_dnf_unset, DNF_UNSET)

inline void lcd_set_cursor(uint8_t row, uint8_t col) {
    new_cmd(0);

    cmd_list->cmd = row == 0 ? CMD((0x80 | (col&0x0F)))
            : CMD((0xC0 | (col&0x0F)));
}

inline void lcd_print_c(char c) {
    new_cmd(LCD_DATA(c));
}

inline void lcd_print(const char* str) {
    for (char* i = str; *i != '\0'; i++) {
        new_cmd(LCD_DATA(*i));
    }
}

inline void lcd_print_int(int val) {
    char integer[16] = { 0 };
    sprintf(integer, "%d", val);
    lcd_print(integer);
}

inline void lcd_print_float(float val) {
    char integer[16] = { 0 };
    sprintf(integer, "%f", val);
    lcd_print(integer);
}

inline void lcd_start_payload() {
    cmd_list = calloc(1, sizeof(lcd_cmd));
}

void lcd_stop_payload() {
    lcd_cmd* i = cmd_list;
    cmd_buff_length = 0;
    while(i->next != NULL) {
        cmd_buff_length += 4;
        i = i->next;
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

inline void lcd_send_payload() {
    HAL_I2C_Master_Transmit_DMA(i2c_handle, lcd_addr, (uint8_t*)cmd_buff, sizeof(uint32_t)*cmd_buff_length);
}

void lcd_init(I2C_HandleTypeDef* handle, uint8_t addr) {
    i2c_handle = handle;
    lcd_addr = addr << 1;

    lcd_start_payload();
    lcd_dnf_unset();
    lcd_dnf_set();
    lcd_set_on(false, false, false);
    lcd_clear_display();
    lcd_set_ddram_increment();
    lcd_set_on(true, false, false);
    lcd_stop_payload();
    lcd_send_payload();
}