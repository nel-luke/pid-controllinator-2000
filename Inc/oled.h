#ifndef OLED_H
#define OLED_H

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

/**
 * This OLED LCD uses I2C for communication
 *
 * Library features functions for drawing lines, rectangles and circles.
 *
 * It also allows you to draw texts and characters using appropriate functions provided in library.
 *
 * Default pinout
 *
OLED    |STM32F10x    |DESCRIPTION

VCC        |3.3V         |
GND        |GND          |
SCL        |PB6          |Serial clock line
SDA        |PB7          |Serial data line
 */

#include "stm32f4xx_hal.h"

#include "fonts.h"

/* I2C address */
#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR         0x78
//#define OLED_I2C_ADDR       0x7A
#endif

/* OLED settings */
/* OLED width in pixels */
#ifndef OLED_WIDTH
#define OLED_WIDTH            128
#endif
/* OLED LCD height in pixels */
#ifndef OLED_HEIGHT
#define OLED_HEIGHT           64
#endif

/**
 * @brief  OLED color enumeration
 */
typedef enum {
    OLED_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
    OLED_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
} OLED_COLOR_t;


/**
 * @brief  Initializes OLED LCD
 * @param  None
 * @retval Initialization status:
 *           - 0: LCD was not detected on I2C port
 *           - > 0: LCD initialized OK and ready to use
 */
HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *handle);

/** 
 * @brief  Updates buffer from internal RAM to LCD
 * @note   This function must be called each time you do some changes to LCD, to update buffer from RAM to LCD
 * @param  None
 * @retval None
 */
void OLED_UpdateScreen(void);

/**
 * @brief  Toggles pixels invertion inside internal RAM
 * @note   @ref OLED_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  None
 * @retval None
 */
void OLED_ToggleInvert(void);

/** 
 * @brief  Fills entire LCD with desired color
 * @note   @ref OLED_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  Color: Color to be used for screen fill. This parameter can be a value of @ref OLED_COLOR_t enumeration
 * @retval None
 */
void OLED_Fill(OLED_COLOR_t Color);

/**
 * @brief  Draws pixel at desired location
 * @note   @ref OLED_UpdateScreen() must called after that in order to see updated LCD screen
 * @param  x: X location. This parameter can be a value between 0 and OLED_WIDTH - 1
 * @param  y: Y location. This parameter can be a value between 0 and OLED_HEIGHT - 1
 * @param  color: Color to be used for screen fill. This parameter can be a value of @ref OLED_COLOR_t enumeration
 * @retval None
 */
void OLED_DrawPixel(uint16_t x, uint16_t y, OLED_COLOR_t color);

/**
 * @brief  Sets cursor pointer to desired location for strings
 * @param  x: X location. This parameter can be a value between 0 and OLED_WIDTH - 1
 * @param  y: Y location. This parameter can be a value between 0 and OLED_HEIGHT - 1
 * @retval None
 */
void OLED_GotoXY(uint16_t x, uint16_t y);

/**
 * @brief  Puts character to internal RAM
 * @note   @ref OLED_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  ch: Character to be written
 * @param  *Font: Pointer to @ref FontDef_t structure with used font
 * @param  color: Color used for drawing. This parameter can be a value of @ref OLED_COLOR_t enumeration
 * @retval Character written
 */
char OLED_Putc(char ch, FontDef_t *Font, OLED_COLOR_t color);

/**
 * @brief  Puts string to internal RAM
 * @note   @ref OLED_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  *str: String to be written
 * @param  *Font: Pointer to @ref FontDef_t structure with used font
 * @param  color: Color used for drawing. This parameter can be a value of @ref OLED_COLOR_t enumeration
 * @retval Zero on success or character value when function failed
 */
char OLED_Puts(char *str, FontDef_t *Font, OLED_COLOR_t color);

/**
 * @brief  Draws line on LCD
 * @note   @ref OLED_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x0: Line X start point. Valid input is 0 to OLED_WIDTH - 1
 * @param  y0: Line Y start point. Valid input is 0 to OLED_HEIGHT - 1
 * @param  x1: Line X end point. Valid input is 0 to OLED_WIDTH - 1
 * @param  y1: Line Y end point. Valid input is 0 to OLED_HEIGHT - 1
 * @param  c: Color to be used. This parameter can be a value of @ref OLED_COLOR_t enumeration
 * @retval None
 */
void OLED_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, OLED_COLOR_t c);

/**
 * @brief  Draws rectangle on LCD
 * @note   @ref OLED_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: Top left X start point. Valid input is 0 to OLED_WIDTH - 1
 * @param  y: Top left Y start point. Valid input is 0 to OLED_HEIGHT - 1
 * @param  w: Rectangle width in units of pixels
 * @param  h: Rectangle height in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref OLED_COLOR_t enumeration
 * @retval None
 */
void OLED_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, OLED_COLOR_t c);

/**
 * @brief  Draws filled rectangle on LCD
 * @note   @ref OLED_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: Top left X start point. Valid input is 0 to OLED_WIDTH - 1
 * @param  y: Top left Y start point. Valid input is 0 to OLED_HEIGHT - 1
 * @param  w: Rectangle width in units of pixels
 * @param  h: Rectangle height in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref OLED_COLOR_t enumeration
 * @retval None
 */
void OLED_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, OLED_COLOR_t c);

/**
 * @brief  Draws triangle on LCD
 * @note   @ref OLED_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x1: First coordinate X location. Valid input is 0 to OLED_WIDTH - 1
 * @param  y1: First coordinate Y location. Valid input is 0 to OLED_HEIGHT - 1
 * @param  x2: Second coordinate X location. Valid input is 0 to OLED_WIDTH - 1
 * @param  y2: Second coordinate Y location. Valid input is 0 to OLED_HEIGHT - 1
 * @param  x3: Third coordinate X location. Valid input is 0 to OLED_WIDTH - 1
 * @param  y3: Third coordinate Y location. Valid input is 0 to OLED_HEIGHT - 1
 * @param  c: Color to be used. This parameter can be a value of @ref OLED_COLOR_t enumeration
 * @retval None
 */
void OLED_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3,
                          OLED_COLOR_t color);

/**
 * @brief  Draws circle to STM buffer
 * @note   @ref OLED_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: X location for center of circle. Valid input is 0 to OLED_WIDTH - 1
 * @param  y: Y location for center of circle. Valid input is 0 to OLED_HEIGHT - 1
 * @param  r: Circle radius in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref OLED_COLOR_t enumeration
 * @retval None
 */
void OLED_DrawCircle(int16_t x0, int16_t y0, int16_t r, OLED_COLOR_t c);

/**
 * @brief  Draws filled circle to STM buffer
 * @note   @ref OLED_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: X location for center of circle. Valid input is 0 to OLED_WIDTH - 1
 * @param  y: Y location for center of circle. Valid input is 0 to OLED_HEIGHT - 1
 * @param  r: Circle radius in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref OLED_COLOR_t enumeration
 * @retval None
 */
void OLED_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, OLED_COLOR_t c);

/**
 * @brief  Initializes OLED LCD
 * @param  None
 * @retval Initialization status:
 *           - 0: LCD was not detected on I2C port
 *           - > 0: LCD initialized OK and ready to use
 */
void OLED_I2C_Init();

/**
 * @brief  Draws the Bitmap
 * @param  X:  X location to start the Drawing
 * @param  Y:  Y location to start the Drawing
 * @param  *bitmap : Pointer to the bitmap
 * @param  W : width of the image
 * @param  H : Height of the image
 * @param  color : 1-> white/blue, 0-> black
 */
void OLED_DrawBitmap(int16_t x, int16_t y, const unsigned char *bitmap, int16_t w, int16_t h, uint16_t color);

// scroll the screen for fixed rows

void OLED_ScrollRight(uint8_t start_row, uint8_t end_row);


void OLED_ScrollLeft(uint8_t start_row, uint8_t end_row);


void OLED_Scrolldiagright(uint8_t start_row, uint8_t end_row);


void OLED_Scrolldiagleft(uint8_t start_row, uint8_t end_row);


void OLED_Stopscroll(void);


// inverts the display i = 1->inverted, i = 0->normal
void OLED_InvertDisplay(int i);


// clear the display
void OLED_Clear(void);


/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
