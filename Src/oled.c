#include "oled.h"

#include <malloc.h>

I2C_HandleTypeDef *i2c_handle = NULL;

static uint8_t data_buffer[OLED_WIDTH + 1];

static void OLED_send(const uint8_t *data, uint8_t type, size_t size) {
    while (HAL_I2C_GetState(i2c_handle) != HAL_I2C_STATE_READY) {
        HAL_Delay(1);
    }

    data_buffer[0] = type;
    memcpy(data_buffer + 1, data, size);
    HAL_I2C_Master_Transmit_DMA(i2c_handle, OLED_I2C_ADDR, data_buffer, size + 1);
}

#define OLED_SEND_COMMANDS(data, size)   OLED_send(data, 0x00, size)
#define OLED_SEND_DATA(data, size)       OLED_send(data, 0x40, size)

/* Absolute value */
#define ABS(x)   ((x) > 0 ? (x) : -(x))

/* OLED data buffer */
static uint8_t OLED_Buffer[OLED_WIDTH * OLED_HEIGHT / 8];

/* Private OLED structure */
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} OLED_t;

/* Private variable */
static OLED_t OLED = {0};


#define OLED_RIGHT_HORIZONTAL_SCROLL              0x26
#define OLED_LEFT_HORIZONTAL_SCROLL               0x27
#define OLED_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define OLED_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A
#define OLED_DEACTIVATE_SCROLL                    0x2E // Stop scroll
#define OLED_ACTIVATE_SCROLL                      0x2F // Start scroll
#define OLED_SET_VERTICAL_SCROLL_AREA             0xA3 // Set scroll range

#define OLED_NORMALDISPLAY       0xA6
#define OLED_INVERTDISPLAY       0xA7


void OLED_ScrollRight(uint8_t start_row, uint8_t end_row) {
    uint8_t commands[] = {
            OLED_RIGHT_HORIZONTAL_SCROLL,
            0x00,  // send dummy
            start_row,  // start page address
            0X00,  // time interval 5 frames
            end_row,  // end page address
            0X00,
            0XFF,
            OLED_ACTIVATE_SCROLL // start scroll
    };
    OLED_SEND_COMMANDS(commands, sizeof(commands));
}


void OLED_ScrollLeft(uint8_t start_row, uint8_t end_row) {
    uint8_t commands[] = {
            OLED_LEFT_HORIZONTAL_SCROLL,  // send 0x26
            0x00,  // send dummy
            start_row,  // start page address
            0X00,  // time interval 5 frames
            end_row,  // end page address
            0X00,
            0XFF,
            OLED_ACTIVATE_SCROLL // start scroll
    };
    OLED_SEND_COMMANDS(commands, sizeof(commands));
}


void OLED_Scrolldiagright(uint8_t start_row, uint8_t end_row) {
    uint8_t commands[] = {
            OLED_SET_VERTICAL_SCROLL_AREA,  // sect the area
            0x00,   // write dummy
            OLED_HEIGHT,
            OLED_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL,
            0x00,
            start_row,
            0X00,
            end_row,
            0x01,
            OLED_ACTIVATE_SCROLL
    };
    OLED_SEND_COMMANDS(commands, sizeof(commands));
}


void OLED_Scrolldiagleft(uint8_t start_row, uint8_t end_row) {
    uint8_t commands[] = {
            OLED_SET_VERTICAL_SCROLL_AREA,  // sect the area
            0x00,   // write dummy
            OLED_HEIGHT,

            OLED_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL,
            0x00,
            start_row,
            0X00,
            end_row,
            0x01,
            OLED_ACTIVATE_SCROLL
    };
    OLED_SEND_COMMANDS(commands, sizeof(commands));
}


void OLED_Stopscroll(void) {
    uint8_t commands[] = {
            OLED_DEACTIVATE_SCROLL
    };
    OLED_SEND_COMMANDS(commands, sizeof(commands));
}


void OLED_InvertDisplay(int i) {
    uint8_t command = 0;
    if (i)
        command = OLED_INVERTDISPLAY;
    else
        command = OLED_NORMALDISPLAY;

    OLED_SEND_COMMANDS(&command, 1);
}

HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *handle) {
    i2c_handle = handle;

    /* Check if OLED is already initialized */
    if (OLED.Initialized == 1)
        return HAL_OK;

    /* Check if LCD connected to I2C */
    if (HAL_I2C_IsDeviceReady(i2c_handle, OLED_I2C_ADDR, 1, 20000) != HAL_OK) {
        /* Return false */
        return HAL_ERROR;
    }

    /* Init LCD */
    uint8_t commands[] = {
            0xAE, //display off
            0x20, //Set Memory Addressing Mode
            0x10, //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing RESET,11,Invalid
            0xB0, //Set Page Start Address for Page Addressing Mode,0-7
            0xC8, //Set COM Output Scan Direction
            0x00, //---set low column address
            0x10, //---set high column address
            0x40, //--set start line address
            0x81, //--set contrast control register
            0xFF,
            0xA1, //--set segment re-map 0 to 127
            0xA6, //--set normal display
            0xA8, //--set multiplex 1 to 64)
            0x3F, //
            0xA4, //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
            0xD3, //-set display offset
            0x00, //-not offset
            0xD5, //--set display clock divide ratio/oscillator frequency
            0xF0, //--set divide ratio
            0xD9, //--set pre-charge period
            0x22, //
            0xDA, //--set com pins hardware configuration
            0x12,
            0xDB, //--set vcomh
            0x20, //0x20,0.77xVcc
            0x8D, //--set DC-DC enable
            0x14, //
            OLED_DEACTIVATE_SCROLL,
            0xAF //--turn on OLED panel
    };

    OLED_SEND_COMMANDS(commands, sizeof(commands));

    /* Clear screen */
    OLED_Fill(OLED_COLOR_BLACK);

    /* Update screen */
    OLED_UpdateScreen();

    /* Set default values */
    OLED.CurrentX = 0;
    OLED.CurrentY = 0;

    /* Initialized OK */
    OLED.Initialized = 1;

    /* Return OK */
    return HAL_OK;
}

void OLED_UpdateScreen(void) {
    uint8_t m;

    uint8_t commands[] = {0xB0, 0x00, 0x10};
    for (m = 0; m < 8; m++) {
        commands[0] = 0xB0 + m;

        OLED_SEND_COMMANDS(commands, sizeof(commands));
        OLED_SEND_DATA(&OLED_Buffer[OLED_WIDTH * m], OLED_WIDTH);
    }

}

void OLED_ON(void) {
    uint8_t commands[] = {
            0x8D,
            0x14,
            0xAF
    };
    OLED_SEND_COMMANDS(commands, sizeof(commands));
}

void OLED_OFF(void) {
    uint8_t commands[] = {
            0x8D,
            0x10,
            0xAE
    };
    OLED_SEND_COMMANDS(commands, sizeof(commands));
}

// -- Framebuffer operations -- //

void OLED_ToggleInvert(void) {
    uint16_t i;

    /* Toggle invert */
    OLED.Inverted = !OLED.Inverted;

    /* Do memory toggle */
    for (i = 0; i < sizeof(OLED_Buffer); i++) {
        OLED_Buffer[i] = ~OLED_Buffer[i];
    }
}

void OLED_DrawBitmap(int16_t x, int16_t y, const unsigned char *bitmap, int16_t w, int16_t h, uint16_t color) {

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for (int16_t j = 0; j < h; j++, y++) {
        for (int16_t i = 0; i < w; i++) {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = (*(const unsigned char *) (&bitmap[j * byteWidth + i / 8]));
            }
            if (byte & 0x80) OLED_DrawPixel(x + i, y, color);
        }
    }
}

void OLED_Fill(OLED_COLOR_t color) {
    /* Set memory */
    memset(OLED_Buffer, (color == OLED_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(OLED_Buffer));
}

void OLED_DrawPixel(uint16_t x, uint16_t y, OLED_COLOR_t color) {
    if (
            x >= OLED_WIDTH ||
            y >= OLED_HEIGHT
            ) {
        /* Error */
        return;
    }

    /* Check if pixels are inverted */
    if (OLED.Inverted) {
        color = (OLED_COLOR_t) !color;
    }

    /* Set color */
    if (color == OLED_COLOR_WHITE) {
        OLED_Buffer[x + (y / 8) * OLED_WIDTH] |= 1 << (y % 8);
    } else {
        OLED_Buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
}

void OLED_GotoXY(uint16_t x, uint16_t y) {
    /* Set write pointers */
    OLED.CurrentX = x;
    OLED.CurrentY = y;
}

char OLED_Putc(char ch, FontDef_t *Font, OLED_COLOR_t color) {
    uint32_t i, b, j;

    /* Check available space in LCD */
    if (
            OLED_WIDTH <= (OLED.CurrentX + Font->FontWidth) ||
            OLED_HEIGHT <= (OLED.CurrentY + Font->FontHeight)
            ) {
        /* Error */
        return 0;
    }

    /* Go through font */
    for (i = 0; i < Font->FontHeight; i++) {
        b = Font->data[(ch - 32) * Font->FontHeight + i];
        for (j = 0; j < Font->FontWidth; j++) {
            if ((b << j) & 0x8000) {
                OLED_DrawPixel(OLED.CurrentX + j, (OLED.CurrentY + i), (OLED_COLOR_t) color);
            } else {
                OLED_DrawPixel(OLED.CurrentX + j, (OLED.CurrentY + i), (OLED_COLOR_t) !color);
            }
        }
    }

    /* Increase pointer */
    OLED.CurrentX += Font->FontWidth;

    /* Return character written */
    return ch;
}

char OLED_Puts(char *str, FontDef_t *Font, OLED_COLOR_t color) {
    /* Write characters */
    while (*str) {
        /* Write character by character */
        if (OLED_Putc(*str, Font, color) != *str) {
            /* Return error */
            return *str;
        }

        /* Increase string pointer */
        str++;
    }

    /* Everything OK, zero should be returned */
    return *str;
}


void OLED_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, OLED_COLOR_t c) {
    int16_t dx, dy, sx, sy, err, e2, i, tmp;

    /* Check for overflow */
    if (x0 >= OLED_WIDTH) {
        x0 = OLED_WIDTH - 1;
    }
    if (x1 >= OLED_WIDTH) {
        x1 = OLED_WIDTH - 1;
    }
    if (y0 >= OLED_HEIGHT) {
        y0 = OLED_HEIGHT - 1;
    }
    if (y1 >= OLED_HEIGHT) {
        y1 = OLED_HEIGHT - 1;
    }

    dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
    dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;
    err = ((dx > dy) ? dx : -dy) / 2;

    if (dx == 0) {
        if (y1 < y0) {
            tmp = y1;
            y1 = y0;
            y0 = tmp;
        }

        if (x1 < x0) {
            tmp = x1;
            x1 = x0;
            x0 = tmp;
        }

        /* Vertical line */
        for (i = y0; i <= y1; i++) {
            OLED_DrawPixel(x0, i, c);
        }

        /* Return from function */
        return;
    }

    if (dy == 0) {
        if (y1 < y0) {
            tmp = y1;
            y1 = y0;
            y0 = tmp;
        }

        if (x1 < x0) {
            tmp = x1;
            x1 = x0;
            x0 = tmp;
        }

        /* Horizontal line */
        for (i = x0; i <= x1; i++) {
            OLED_DrawPixel(i, y0, c);
        }

        /* Return from function */
        return;
    }

    while (1) {
        OLED_DrawPixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        e2 = err;
        if (e2 > -dx) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dy) {
            err += dx;
            y0 += sy;
        }
    }
}

void OLED_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, OLED_COLOR_t c) {
    /* Check input parameters */
    if (
            x >= OLED_WIDTH ||
            y >= OLED_HEIGHT
            ) {
        /* Return error */
        return;
    }

    /* Check width and height */
    if ((x + w) >= OLED_WIDTH) {
        w = OLED_WIDTH - x;
    }
    if ((y + h) >= OLED_HEIGHT) {
        h = OLED_HEIGHT - y;
    }

    /* Draw 4 lines */
    OLED_DrawLine(x, y, x + w, y, c);         /* Top line */
    OLED_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
    OLED_DrawLine(x, y, x, y + h, c);         /* Left line */
    OLED_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}

void OLED_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, OLED_COLOR_t c) {
    uint8_t i;

    /* Check input parameters */
    if (
            x >= OLED_WIDTH ||
            y >= OLED_HEIGHT
            ) {
        /* Return error */
        return;
    }

    /* Check width and height */
    if ((x + w) >= OLED_WIDTH) {
        w = OLED_WIDTH - x;
    }
    if ((y + h) >= OLED_HEIGHT) {
        h = OLED_HEIGHT - y;
    }

    /* Draw lines */
    for (i = 0; i <= h; i++) {
        /* Draw lines */
        OLED_DrawLine(x, y + i, x + w, y + i, c);
    }
}

void OLED_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3,
                          OLED_COLOR_t color) {
    /* Draw lines */
    OLED_DrawLine(x1, y1, x2, y2, color);
    OLED_DrawLine(x2, y2, x3, y3, color);
    OLED_DrawLine(x3, y3, x1, y1, color);
}


void OLED_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3,
                                OLED_COLOR_t color) {
    int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
            yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
            curpixel = 0;

    deltax = ABS(x2 - x1);
    deltay = ABS(y2 - y1);
    x = x1;
    y = y1;

    if (x2 >= x1) {
        xinc1 = 1;
        xinc2 = 1;
    } else {
        xinc1 = -1;
        xinc2 = -1;
    }

    if (y2 >= y1) {
        yinc1 = 1;
        yinc2 = 1;
    } else {
        yinc1 = -1;
        yinc2 = -1;
    }

    if (deltax >= deltay) {
        xinc1 = 0;
        yinc2 = 0;
        den = deltax;
        num = deltax / 2;
        numadd = deltay;
        numpixels = deltax;
    } else {
        xinc2 = 0;
        yinc1 = 0;
        den = deltay;
        num = deltay / 2;
        numadd = deltax;
        numpixels = deltay;
    }

    for (curpixel = 0; curpixel <= numpixels; curpixel++) {
        OLED_DrawLine(x, y, x3, y3, color);

        num += numadd;
        if (num >= den) {
            num -= den;
            x += xinc1;
            y += yinc1;
        }
        x += xinc2;
        y += yinc2;
    }
}

void OLED_DrawCircle(int16_t x0, int16_t y0, int16_t r, OLED_COLOR_t c) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    OLED_DrawPixel(x0, y0 + r, c);
    OLED_DrawPixel(x0, y0 - r, c);
    OLED_DrawPixel(x0 + r, y0, c);
    OLED_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        OLED_DrawPixel(x0 + x, y0 + y, c);
        OLED_DrawPixel(x0 - x, y0 + y, c);
        OLED_DrawPixel(x0 + x, y0 - y, c);
        OLED_DrawPixel(x0 - x, y0 - y, c);

        OLED_DrawPixel(x0 + y, y0 + x, c);
        OLED_DrawPixel(x0 - y, y0 + x, c);
        OLED_DrawPixel(x0 + y, y0 - x, c);
        OLED_DrawPixel(x0 - y, y0 - x, c);
    }
}

void OLED_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, OLED_COLOR_t c) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    OLED_DrawPixel(x0, y0 + r, c);
    OLED_DrawPixel(x0, y0 - r, c);
    OLED_DrawPixel(x0 + r, y0, c);
    OLED_DrawPixel(x0 - r, y0, c);
    OLED_DrawLine(x0 - r, y0, x0 + r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        OLED_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
        OLED_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

        OLED_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
        OLED_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
    }
}


void OLED_Clear(void) {
    OLED_Fill(0);
    OLED_UpdateScreen();
}