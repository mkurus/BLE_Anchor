/*
 * i2c_main.h
 *
 *  Created on: 28 May 2018
 *      Author: TrioMobil - Murat
 */

#ifndef PROJECTS_BLE_ANCHOR_I2C_MAIN_H_
#define PROJECTS_BLE_ANCHOR_I2C_MAIN_H_

#include <i2c.h>
#include <i2c_hal.h>

#define EEMPROM_SLAVE_ADDR (0x30u >> 1u)

void EEPROM_WriteCallback(Driver_Status_t status, void *pUserData);
void EEPROM_ReadCallback(Driver_Status_t status, void *pUserData);
void I2C_BlockingDelay_us(uint16_t us);
int I2C_ReadBlocking(uint8_t i2cDevice, uint8_t *pBuffer, uint8_t bytes, Driver_Callback_t callbackFunction, void *pUserData);
int I2C_WriteBlockingInLine(uint8_t i2cDevice, uint8_t *pBuffer, uint8_t bytes,	Driver_Callback_t callbackFunction, void *pUserData);
int I2C_ReadData_Blocking(uint8_t address, uint8_t *rxBuffPtr);
int I2C_WriteData_Blocking(uint8_t address, uint8_t *rxBuffPtr);

#endif /* PROJECTS_BLE_ANCHOR_I2C_MAIN_H_ */
