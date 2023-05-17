//
// Created by Luke Nel on 17/05/2023.
//

#include "controller.h"
#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "lcd.h"
#include "eeprom.h"
#include "oled.h"

#define ADC_BUFF_LEN 2
volatile uint16_t adc_buff[ADC_BUFF_LEN];
volatile bool done = false;

void Controller_Init(void) {
// Check UART
    printf("PID Controllinator 2000\r\n");

    HAL_Delay(100);
    if (LCD_Init(&hi2c1) == HAL_OK) {
        printf("LCD initialized");
    } else {
        printf("!!Error initializing LCD");
    }
    printf(" on I2C1, address 0x%2x.\r\n", LCD_I2C_ADDR);

    if (EEPROM_Init(&hi2c1) == HAL_OK) {
        printf("EEPROM initialized");
    } else {
        printf("!!Error initializing EEPROM");
    }
    printf(" on I2C1, address 0x%2x.\r\n", EEPROM_I2C_ADDR);

    if (OLED_Init(&hi2c2) == HAL_OK) {
        printf("OLED initialized");
    } else {
        printf("!!Error initializing OLED");
    }
    printf(" on I2C2, address 0x%2x.\r\n", OLED_I2C_ADDR);

    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) == HAL_OK) {
        printf("Rotary encoder initialized");
    } else {
        printf("!!Error initializing rotary encoder");
    }
    printf(" on TIM2.\r\n");

    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Buzzer

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) == HAL_OK) {
        printf("PWM output initialized");
    } else {
        printf("!!Error initializing");
    }
    printf(" on TIM3.\r\n");

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buff, ADC_BUFF_LEN)
        == HAL_OK) {
        printf("ADC initialized.\r\n");
    } else {
        printf("!!Error initializing ADC.\r\n");
    }
}

void Controller_Welcome(void) {

    HAL_GPIO_WritePin(ControlLED_GPIO_Port, ControlLED_Pin, GPIO_PIN_SET);

    LCD_Begin_Payload();
    LCD_Clear_Display();
    LCD_Enable(true, false, false);
    LCD_Set_Cursor(0, 0);
    LCD_Print_s("PID");
    LCD_Set_Cursor(1, 1);
    LCD_Print_s("Controllinator");
    LCD_Set_Cursor(0, 12);
    LCD_Print_i(2000);
    LCD_End_Payload();
    LCD_Send_Payload();

    OLED_GotoXY(20, 22);
    OLED_Puts("P.I.D", &Font_16x26, 1);
    OLED_UpdateScreen();

    HAL_Delay(2000);

    LCD_Begin_Payload();
    LCD_Clear_Display();
    LCD_End_Payload();
    LCD_Send_Payload();

    OLED_Clear();

//    int i = 0;
//    for (;;) {
//        HAL_Delay(500);
//        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//
//        int enc_val = (int) (TIM2->CNT >> 2);
//
//        //HAL_ADC_Start_DMA(&hadc1, adc_buff, sizeof(adc_buff));
//
//        //osDelay(50);
////        ADC1->CR2 &= ~ADC_CR2_DMA;
//
//        //memset(adc_buff, 0, sizeof(adc_buff) * sizeof(uint32_t));
////        ADC1->CR2 |= ADC_CR2_DMA;
//        if (done) {
//            done = false;
//            printf("%3d: ", i++);
//            for (int j = 0; j < ADC_BUFF_LEN; j++) {
//                float val = (float) (adc_buff[j] >> 4) / 1240;
//                printf("%0.3f ", val);
//            }
//            printf("\r\n");
//            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buff, ADC_BUFF_LEN);
//        }
//    }
}