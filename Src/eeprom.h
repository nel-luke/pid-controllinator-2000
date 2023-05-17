//
// Created by Luke Nel on 13/05/2023.
//

#ifndef PID_CONTROLLER_EEPROM_H
#define PID_CONTROLLER_EEPROM_H

#include "stm32f4xx_hal.h"

/* I2C address */
#ifndef EEPROM_I2C_ADDR
#define EEPROM_I2C_ADDR         0xA0
#endif

HAL_StatusTypeDef EEPROM_Init(I2C_HandleTypeDef* handle);

#endif //PID_CONTROLLER_EEPROM_H
