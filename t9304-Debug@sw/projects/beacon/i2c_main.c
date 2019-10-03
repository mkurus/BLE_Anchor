/*
 * i2c_main.c
 *
 *  Created on: 28 May 2018
 *      Author: TrioMobil - Murat
 */

#include <i2c.h>
#include <i2c_hal.h>
#include "i2c_main.h"

uint8_t I2C_Buffer;
bool FLAG_ReadCallBack;
bool FLAG_WriteCallBack;

/***********************************************************************
 *
 * EEPROM_WriteCallback
 *
 ***********************************************************************/
void EEPROM_WriteCallback(Driver_Status_t status, void *pUserData)
{
	if(DRIVER_STATUS_OK == status)
		FLAG_WriteCallBack = true;
	else
		FLAG_WriteCallBack = false;
}

/***********************************************************************
 *
 * EEPROM_ReadCallback
 *
 ***********************************************************************/
void EEPROM_ReadCallback(Driver_Status_t status, void *pUserData)
{
	if(DRIVER_STATUS_OK == status)
		FLAG_ReadCallBack = true;
	else
		FLAG_ReadCallBack = false;
}

/***********************************************************************
 *
 * I2C_Delay
 *
 ***********************************************************************/
void I2C_BlockingDelay_us(uint16_t us)
{
	uint16_t delay = us * 5;				// delay value must be bigger than 5 times of the us value

	for(int i=0;i<delay;i++)
		asm("NOP");
}

/***********************************************************************
 *
 *  I2C Read Blocking
 *
 ***********************************************************************/
int I2C_ReadBlocking(uint8_t i2cDevice, uint8_t *pBuffer, uint8_t bytes,
		Driver_Callback_t callbackFunction, void *pUserData)
{
	FLAG_ReadCallBack = false;
	(void)I2C_ReadBytes(i2cDevice, (uint8_t*)pBuffer,
			bytes, callbackFunction, pUserData);

	uint16_t timeout = (1000 * bytes);		// 60
	do{
		if(FLAG_ReadCallBack)
			return 1;
		I2C_BlockingDelay_us(5);
	}
	while(--timeout > 0);
	return 0;
}

/***********************************************************************
 *
 *  I2C Write Blocking
 *
 ***********************************************************************/
int I2C_WriteBlockingInLine(uint8_t i2cDevice, uint8_t *pBuffer, uint8_t bytes,
		Driver_Callback_t callbackFunction, void *pUserData)
{
	FLAG_WriteCallBack = false;
	(void)I2C_WriteBytesInline(i2cDevice, (uint8_t*)pBuffer,
			bytes, callbackFunction, pUserData);

	uint16_t timeout = (1000 * bytes); 		// 60
	do{
		if(FLAG_WriteCallBack)
			return 1;
		I2C_BlockingDelay_us(5);
	}
	while(--timeout > 0);
	return 0;
}


/***********************************************************************
 *
 * I2C ReadData Blocking
 *
 ***********************************************************************/
int I2C_ReadData_Blocking(uint8_t address, uint8_t *rxBuffPtr)
{
	//......................
	if(!I2C_WriteBlockingInLine(EEMPROM_SLAVE_ADDR, (uint8_t*)&address, 1, EEPROM_WriteCallback, NULL))
	{
		return 2;
	}
	//......................
	if(!I2C_ReadBlocking(EEMPROM_SLAVE_ADDR, (uint8_t*)rxBuffPtr, 1, EEPROM_ReadCallback, NULL))
	{
		return 3;
	}
	//......................
	return 1;
}

/***********************************************************************
 *
 * I2C ReadData Blocking
 *
 ***********************************************************************/
int I2C_WriteData_Blocking(uint8_t address, uint8_t *rxBuffPtr)
{
	uint8_t rxBuffer[2];

	rxBuffer[0] = address;
	rxBuffer[1] = *rxBuffPtr;

	if(!I2C_WriteBlockingInLine(EEMPROM_SLAVE_ADDR, (uint8_t*)&rxBuffer[0], 1+1, EEPROM_WriteCallback, NULL))
	{
		return 0;
	}
	return 1;
}
