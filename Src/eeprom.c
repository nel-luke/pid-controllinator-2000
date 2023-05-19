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
//    uint8_t check = 178;
//    HAL_I2C_Mem_Write(i2c_handle, EEPROM_I2C_ADDR, 0x0010, 2, &check, 1, 10);
//    HAL_Delay(10);
//    uint8_t result = 0;
//    HAL_I2C_Mem_Read(i2c_handle, EEPROM_I2C_ADDR, 0x0010, 2, &result, 1, 10);
//
//    if (result != check)
//        return HAL_ERROR;

    return HAL_OK;
}

void EEPROM_Load(uint16_t addr, void *data, size_t size) {
    while (HAL_I2C_GetState(i2c_handle) != HAL_I2C_STATE_READY) {
        HAL_Delay(1);
    }

    HAL_I2C_Mem_Read(i2c_handle, EEPROM_I2C_ADDR, addr, 2, data, size, 10);
}

void EEPROM_Save(uint16_t addr, void *data, size_t size) {
    while (HAL_I2C_GetState(i2c_handle) != HAL_I2C_STATE_READY) {
        HAL_Delay(1);
    }

    HAL_I2C_Mem_Write(i2c_handle, EEPROM_I2C_ADDR, addr, 2, data, size, 10);
}
