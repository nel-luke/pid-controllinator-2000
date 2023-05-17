//
// Created by Luke Nel on 13/05/2023.
//

#include "eeprom.h"

static I2C_HandleTypeDef* i2c_handle = NULL;

HAL_StatusTypeDef EEPROM_Init(I2C_HandleTypeDef* handle) {
    i2c_handle = handle;

    if (HAL_I2C_IsDeviceReady(i2c_handle, EEPROM_I2C_ADDR, 1, 20000) != HAL_OK) {
        /* Return false */
        return HAL_ERROR;
    }
    // Check EEPROM
//    uint8_t data[] = {16};
//    HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x0010, 2, data, 1, 100);
//    HAL_Delay(100);
//    uint8_t res = 0;
//    HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0x0010, 2, &res, 1, 100);
//    printf("Res = %d\r\n", res);

    return HAL_OK;
}