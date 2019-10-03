/*
 * lis2dh12_main.h
 *
 *  Created on: 28 May 2018
 *      Author: TrioMobil - Murat
 */

#ifndef PROJECTS_BLE_ANCHOR_LIS2DH12_MAIN_H_
#define PROJECTS_BLE_ANCHOR_LIS2DH12_MAIN_H_

#include <types.h>
#include <macros.h>
#include <string.h>
#include "i2c_main.h"
#include "lis2dh12.h"
#include "lis2dh12_platform.h"

/***********************************************************************
 *
 * Lis2dh12 definitions
 *
 ***********************************************************************/
enum{DISABLE = 0, ENABLE};

/********************************************************/
/* @return: register structure address 				   **/
/********************************************************/
Lis2dh12_register *getLocalptr_Lis2dh12register(lis2dh12_devList dev)
{
	return &lis2dh12_register[dev];
}

/********************************************************/
/* @return: register structure address 				   **/
/********************************************************/
Lis2dh12_config *getLocalptr_Lis2dh12config(lis2dh12_devList dev)
{
	return &lis2dh12_config[dev];
}

/********************************************************/
/* @return: output structure address 				   **/
/********************************************************/
Lis2dh12_output *getLocalptr_Lis2dh12Output(lis2dh12_devList dev)
{
	return &lis2dh12_output[dev];
}


/***********************************************************************
 *
 * deviceSettings
 *
 ***********************************************************************/
void deviceSettings(DefProjectSettings *ptrdefProjectSettings)
{
	Lis2dh12_config *ptrlis2dh12_config = getLocalptr_Lis2dh12config(DEV0_LIS);

	#if	1
		ptrlis2dh12_config->dataRate = Hz25;						/* ODR:  Data rate configuration */
		ptrlis2dh12_config->mgSens = Mg64;							/* Mg64 */
		ptrlis2dh12_config->hpFilter = ENABLE;
		ptrlis2dh12_config->intSource = INT_X|INT_Y|INT_Z;
		ptrlis2dh12_config->intDuration = 1;						/* Duration time is measured in N/ODR, where N is the content of the duration register */
		ptrlis2dh12_config->actDuration = 1;						/* Sleep-to-wake, return-to-sleep duration. 1 LSb = (8*1[LSb]+1)/ODR */

		ptrdefProjectSettings->alarmTime = DEF_MOTION_DETEC_ALARM_TIME;				/* sleep while wait int */
		ptrdefProjectSettings->sleepTime = DEF_MOTION_DETEC_SLEEP_TIME;				/* sleep for sample period */
		ptrdefProjectSettings->whyWakeUp = VIAERR;
	#endif

//#if	MOTION_DETECTION
//	ptrlis2dh12_config->dataRate = Hz10;						/* ODR:  Data rate configuration */
//	ptrlis2dh12_config->mgSens = DEF_MOTION_DETEC_MGval;		/* Mg32 */
//	ptrlis2dh12_config->hpFilter = ENABLE;
//	ptrlis2dh12_config->intSource = INT_X|INT_Y|INT_Z;
//	ptrlis2dh12_config->intDuration = 2;						/* Duration time is measured in N/ODR, where N is the content of the duration register */
//	ptrlis2dh12_config->actDuration = 2;						/* Sleep-to-wake, return-to-sleep duration. 1 LSb = (8*1[LSb]+1)/ODR */
//
//	ptrdefProjectSettings->alarmTime = DEF_MOTION_DETEC_ALARM_TIME;				/* sleep while wait int */
//	ptrdefProjectSettings->sleepTime = DEF_MOTION_DETEC_SLEEP_TIME;				/* sleep for sample period */
//	ptrdefProjectSettings->whyWakeUp = VIAERR;
//#endif
//
//#if	MOTIONLESS_DETECTION
//	ptrlis2dh12_config->dataRate = Hz25;						/* ODR:  Data rate configuration */
//	ptrlis2dh12_config->mgSens = DEF_MOTIONLESS_DETEC_MGval;	/* Mg32 */
//	ptrlis2dh12_config->hpFilter = ENABLE;
//	ptrlis2dh12_config->intSource = INT_X|INT_Y|INT_Z;
//	ptrlis2dh12_config->intDuration = 2;						/* Duration time is measured in N/ODR, where N is the content of the duration register */
//	ptrlis2dh12_config->actDuration = 2;						/* Sleep-to-wake, return-to-sleep duration. 1 LSb = (8*1[LSb]+1)/ODR */
//
//	ptrdefProjectSettings->alarmTime = DEF_MOTIONLESS_DETEC_ALARM_TIME;				/* sleep while wait int */
//	ptrdefProjectSettings->sleepTime = DEF_MOTIONLESS_DETEC_SLEEP_TIME;				/* sleep for sample period */
//	ptrdefProjectSettings->whyWakeUp = VIAERR;
//#endif
//
//#if	ANGLE_METER
//	ptrlis2dh12_config->dataRate = Hz50;						/* ODR:  Data rate configuration */
//	ptrlis2dh12_config->mgSens = DEF_ANGLE_METER_MGval;			/* Mg64 */
//	ptrlis2dh12_config->hpFilter = DISABLE;
//	ptrlis2dh12_config->intSource = INT_X|INT_Y;
//	ptrlis2dh12_config->intDuration = 0;						/* Duration time is measured in N/ODR, where N is the content of the duration register */
//	ptrlis2dh12_config->actDuration = 0;						/* Sleep-to-wake, return-to-sleep duration. 1 LSb = (8*1[LSb]+1)/ODR */
//
//	ptrdefProjectSettings->alarmTime = DEF_ANGLE_METER_ALARM_TIME;				/* sleep while wait int */
//	ptrdefProjectSettings->sleepTime = DEF_ANGLE_METER_SLEEP_TIME;				/* sleep for sample period */
//#endif
//
//#if	FREE_FALL_DETECTION
//	ptrlis2dh12_config->dataRate = Hz100;						/* ODR:  Data rate configuration */
//	ptrlis2dh12_config->mgSens = DEF_FREE_FALL_DETEC_MGval;		/* Mg352*/
//	ptrlis2dh12_config->hpFilter = ENABLE;
//	ptrlis2dh12_config->intSource = INT_X|INT_Y|INT_Z|INTs_OR;	//INT_X|INT_Y|INT_Z|INTs_OR;
//	ptrlis2dh12_config->intDuration = 4;						/* Duration time is measured in N/ODR, where N is the content of the duration register */
//	ptrlis2dh12_config->actDuration = 4;						/* Sleep-to-wake, return-to-sleep duration. 1 LSb = (8*1[LSb]+1)/ODR */
//
//#endif

}

/*********************************************************************
 * @param checkCntrl: read and compare the written data.
 * @return Returns non-zero value on successful all controls
**********************************************************************/
enum{BIT0 = 0, BIT1, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7};
uint8_t lis2dh12_setReg_CTRL_REG1(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrLis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	/** Write 47h to CTRL_REG1 (20h) **************************************/
	ptrlis2dh12_register->lis2dh12_CTRL_REG1.Xen = 1;			// X-axis enable. Default value: 1(enb)
	ptrlis2dh12_register->lis2dh12_CTRL_REG1.Yen = 1;			// Y-axis enable. Default value: 1(enb)
	ptrlis2dh12_register->lis2dh12_CTRL_REG1.Zen = 1;			// Z-axis enable. Default value: 1(enb)
	ptrlis2dh12_register->lis2dh12_CTRL_REG1.LPen = 1;			// Low-power mode enable. Def:0	 (0: high-resolution /normal mode, 1: low-power mode)
	ptrlis2dh12_register->lis2dh12_CTRL_REG1.ODR0 = (ptrLis2dh12_config->dataRate >> BIT0)&0x01;	// 0000: Power-down mode
	ptrlis2dh12_register->lis2dh12_CTRL_REG1.ODR1 = (ptrLis2dh12_config->dataRate >> BIT1)&0x01;	// 0001: HR / Normal / Low-power mode (1 Hz)
	ptrlis2dh12_register->lis2dh12_CTRL_REG1.ODR2 = (ptrLis2dh12_config->dataRate >> BIT2)&0x01;	// Low-power mode (50 Hz)
	ptrlis2dh12_register->lis2dh12_CTRL_REG1.ODR3 = (ptrLis2dh12_config->dataRate >> BIT3)&0x01;	// 1000: Low-power mode

	//_myprintf("Write CTRL_REG1: %x\n", ptrlis2dh12_register->lis2dh12_CTRL_REG1);
	ret = lis2dh12_SendData(Lis2dh12_ADDR_CTRL_REG1, (uint8_t *)&ptrlis2dh12_register->lis2dh12_CTRL_REG1, checkCntrl);
	return ret;
}


uint8_t lis2dh12_setReg_CTRL_REG2(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrLis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	/** Write 00h to CTRL_REG2 (21h) **************************************/
	ptrlis2dh12_register->lis2dh12_CTRL_REG2.HP_IA1 = (ptrLis2dh12_config->hpFilter)&0x01;		// High-pass filter enabled for AOI function on Interrupt 1
	ptrlis2dh12_register->lis2dh12_CTRL_REG2.HP_IA2 = 0;										// High-pass filter enabled for AOI function on Interrupt 2
	ptrlis2dh12_register->lis2dh12_CTRL_REG2.HPCLICK = 0;										// High-pass filter enabled for CLICK function.
	ptrlis2dh12_register->lis2dh12_CTRL_REG2.FDS = (ptrLis2dh12_config->hpFilter)&0x01;			// Filtered data selection.
	ptrlis2dh12_register->lis2dh12_CTRL_REG2.HPCF1 = 0;											// High-pass filter cutoff frequency selection
	ptrlis2dh12_register->lis2dh12_CTRL_REG2.HPCF2 = 0;											//
	ptrlis2dh12_register->lis2dh12_CTRL_REG2.HPM0 = 0;											// Reference signal for filtering
	ptrlis2dh12_register->lis2dh12_CTRL_REG2.HPM1 = 0;

	//_myprintf("Write CTRL_REG2: %x\n", ptrlis2dh12_register->lis2dh12_CTRL_REG2);
	ret = lis2dh12_SendData(Lis2dh12_ADDR_CTRL_REG2, (uint8_t *)&ptrlis2dh12_register->lis2dh12_CTRL_REG2, checkCntrl);
	return ret;
}

uint8_t lis2dh12_setReg_CTRL_REG3(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrLis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	/** Write 00h to CTRL_REG3 (22h) **************************************/
	ptrlis2dh12_register->lis2dh12_CTRL_REG3.reserved = 0;		// bit:0 reserved
	ptrlis2dh12_register->lis2dh12_CTRL_REG3.I1_OVERRUN = 0;	// FIFO overrun interrupt on INT1 pin.
	ptrlis2dh12_register->lis2dh12_CTRL_REG3.I1_WTM = 0;		// FIFO watermark interrupt on INT1 pin.
	ptrlis2dh12_register->lis2dh12_CTRL_REG3.VAL0 = 0;			// bit:4 must be "0"
	ptrlis2dh12_register->lis2dh12_CTRL_REG3.I1_ZYXDA = ((ptrLis2dh12_config->int1act>>4)&0x01);	// ZYXDA interrupt on INT1 pin.
	ptrlis2dh12_register->lis2dh12_CTRL_REG3.I1_IA2 = ((ptrLis2dh12_config->int1act>>5)&0x01);	// IA2 interrupt on INT1 pin.
	ptrlis2dh12_register->lis2dh12_CTRL_REG3.I1_IA1 = ((ptrLis2dh12_config->int1act>>6)&0x01);	// IA1 interrupt on INT1 pin.
	ptrlis2dh12_register->lis2dh12_CTRL_REG3.I1_CLICK = 0;		// CLICK interrupt on INT1 pin.

	//_myprintf("Write CTRL_REG3: %x\n", ptrlis2dh12_register->lis2dh12_CTRL_REG3);
	ret = lis2dh12_SendData(Lis2dh12_ADDR_CTRL_REG3, (uint8_t *)&ptrlis2dh12_register->lis2dh12_CTRL_REG3, checkCntrl);
	return ret;
}

uint8_t lis2dh12_setReg_CTRL_REG4(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrLis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	/** Write 80h to CTRL_REG4 (23h) **************************************/
	ptrlis2dh12_register->lis2dh12_CTRL_REG4.SIM = 0;			// SPI serial interface mode selection.
	ptrlis2dh12_register->lis2dh12_CTRL_REG4.ST0 = 0;			// Self-test enable.
	ptrlis2dh12_register->lis2dh12_CTRL_REG4.ST1 = 0;			// (00: self-test disabled, 01:Self test 0, 10:Self test 1)
	ptrlis2dh12_register->lis2dh12_CTRL_REG4.HR  = 0;			// Operating mode selection (1: High-resolution, normal mode, low-power mode)
	ptrlis2dh12_register->lis2dh12_CTRL_REG4.FS0 = 0;			// Full-scale selection. (Def: 00)
	ptrlis2dh12_register->lis2dh12_CTRL_REG4.FS1 = 0;			// ( 00: ±2g;  01: ±4g;  10: ±8g;  11: ±16g )
	ptrlis2dh12_register->lis2dh12_CTRL_REG4.BLE = 0;			// Big/Little Endian data selection.
	ptrlis2dh12_register->lis2dh12_CTRL_REG4.BDU = 0;			// Block data update. Def:0

	//_myprintf("Write CTRL_REG4: %x\n", ptrlis2dh12_register->lis2dh12_CTRL_REG4);
	ret = lis2dh12_SendData(Lis2dh12_ADDR_CTRL_REG4, (uint8_t *)&ptrlis2dh12_register->lis2dh12_CTRL_REG4, checkCntrl);
	return ret;
}

uint8_t lis2dh12_setReg_CTRL_REG5(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrLis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	/** Write 00h to CTRL_REG5 (24h) **************************************/
	ptrlis2dh12_register->lis2dh12_CTRL_REG5.D4D_INT2 = 0;			// 4D enable: 4D detection is enabled on INT2 pin.
	ptrlis2dh12_register->lis2dh12_CTRL_REG5.LIR_INT2 = 0;			// Latch interrupt request on INT2_SRC (35h) register.
	ptrlis2dh12_register->lis2dh12_CTRL_REG5.D4D_INT1 = 0;			// 4D enable: 4D detection is enabled on INT1 pin.
	ptrlis2dh12_register->lis2dh12_CTRL_REG5.LIR_INT1 = 0;			// Latch interrupt request on INT1_SRC (31h).
	ptrlis2dh12_register->lis2dh12_CTRL_REG5.rev1 = 0;				// bit5: reserved
	ptrlis2dh12_register->lis2dh12_CTRL_REG5.rev2 = 0;				// bit6: reserved
	ptrlis2dh12_register->lis2dh12_CTRL_REG5.FIFO_EN = 0;			// FIFO enable. (Def:0)
	ptrlis2dh12_register->lis2dh12_CTRL_REG5.BOOT = 0;				// Reboot memory content. (Def:0)

	//_myprintf("Write CTRL_REG5: %x\n", ptrlis2dh12_register->lis2dh12_CTRL_REG5);
	ret = lis2dh12_SendData(Lis2dh12_ADDR_CTRL_REG5, (uint8_t *)&ptrlis2dh12_register->lis2dh12_CTRL_REG5, checkCntrl);
	return ret;
}

uint8_t lis2dh12_setReg_CTRL_REG6(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrLis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	/** Write 00h to CTRL_REG6 (25h) **************************************/
	ptrlis2dh12_register->lis2dh12_CTRL_REG6.rev1 = 0;				// reserved
	ptrlis2dh12_register->lis2dh12_CTRL_REG6.INT_POLARITY = 1;		// INT1 and INT2 pin polarity. (Def:0) (0:active-high; 1:active-low).
	ptrlis2dh12_register->lis2dh12_CTRL_REG6.rev2 = 0;				// reserved
	ptrlis2dh12_register->lis2dh12_CTRL_REG6.I2_ACT = 0;			// Enable activity interrupt on INT2 pin. (Def:0) (0: disabled; 1: enabled)
	ptrlis2dh12_register->lis2dh12_CTRL_REG6.I2_BOOT = 0;			// Enable boot on INT2 pin. (Def:0) (0: disabled; 1: enabled)
	ptrlis2dh12_register->lis2dh12_CTRL_REG6.I2_IA2 = 0;			// Enable interrupt 2 function on INT2 pin. (Def:0) (0: disabled; 1: enabled)
	ptrlis2dh12_register->lis2dh12_CTRL_REG6.I2_IA1 = 0;			// Enable interrupt 1 function on INT2 pin. (Def:0) (0: disabled; 1: enabled)
	ptrlis2dh12_register->lis2dh12_CTRL_REG6.I2_CLICK = 0;			// Click interrupt on INT2 pin. (Def:0) (0: disabled; 1: enabled)

	//_myprintf("Write CTRL_REG6: %x\n", ptrlis2dh12_register->lis2dh12_CTRL_REG6);
	ret = lis2dh12_SendData(Lis2dh12_ADDR_CTRL_REG6, (uint8_t *)&ptrlis2dh12_register->lis2dh12_CTRL_REG6, checkCntrl);
	return ret;
}

/*********************************************************************
 * @param
 * @return Returns non-zero value on successful all controls
**********************************************************************/
uint8_t lis2dh12_readReg_STATUS(Lis2dh12_register *ptrlis2dh12_register)
{
	if(!lis2dh12_ReadData(Lis2dh12_ADDR_STATUS_REG, &ptrlis2dh12_register->lis2dh12_STATUS_REG._Lis2dh12_STATUS_REG)){
		//_myprintf("problem when write/read STATUS_REG\r\n");
		return 0;
	}

	return 1;
}

/*********************************************************************
 * @param
 * @return Returns non-zero value on successful all controls
**********************************************************************/

uint8_t lis2dh12_readReg_INT1SRC(Lis2dh12_register * ptrlis2dh12_register)
{
	if(!lis2dh12_ReadData(Lis2dh12_ADDR_INT1_SRC, &ptrlis2dh12_register->lis2dh12_INT1SRC._Lis2dh12_INT1SRC)){
		return 0;
	}

	return 1;
}

/*********************************************************************
 * @param
 * @return Returns non-zero value on successful all controls
**********************************************************************/
uint8_t lis2dh12_readReg_XYZ(Lis2dh12_register *ptrlis2dh12_register)
{
	if(!lis2dh12_ReadData(Lis2dh12_ADDR_OUT_X_L, (uint8_t *)&ptrlis2dh12_register->lis2dh12_OUT_X.OUT_X_L)){
		//_myprintf("problem when write/read OUT_X_L\r\n");
		return 0;
	}

	if(!lis2dh12_ReadData(Lis2dh12_ADDR_OUT_X_H, (uint8_t *)&ptrlis2dh12_register->lis2dh12_OUT_X.OUT_X_H)){
		//_myprintf("problem when write/read OUT_X_H\r\n");
		return 0;
	}

	if(!lis2dh12_ReadData(Lis2dh12_ADDR_OUT_Y_L, (uint8_t *)&ptrlis2dh12_register->lis2dh12_OUT_Y.OUT_Y_L)){
		//_myprintf("problem when write/read OUT_Y_L\r\n");
		return 0;
	}

	if(!lis2dh12_ReadData(Lis2dh12_ADDR_OUT_Y_H, (uint8_t *)&ptrlis2dh12_register->lis2dh12_OUT_Y.OUT_Y_H)){
		//_myprintf("problem when write/read OUT_Y_H\r\n");
		return 0;
	}

	if(!lis2dh12_ReadData(Lis2dh12_ADDR_OUT_Z_L, (uint8_t *)&ptrlis2dh12_register->lis2dh12_OUT_Z.OUT_Z_L)){
		//_myprintf("problem when write/read OUT_Z_L\r\n");
		return 0;
	}

	if(!lis2dh12_ReadData(Lis2dh12_ADDR_OUT_Z_H, (uint8_t *)&ptrlis2dh12_register->lis2dh12_OUT_Z.OUT_Z_H)){
		//_myprintf("problem when write/read OUT_Z_H\r\n");
		return 0;
	}

	return 1;
}

/*********************************************************************
 * @param
 * @return Returns old CTRL_REG1 value
**********************************************************************/
uint8_t lis2dh12_powerDownMode(Lis2dh12_register *ptrlis2dh12_register)
{
	uint8_t oldReg1Value;
	if(!lis2dh12_ReadData(Lis2dh12_ADDR_CTRL_REG1, &ptrlis2dh12_register->lis2dh12_CTRL_REG1._Lis2dh12_CTRL_REG1)){
		//_myprintf("problem when write/read STATUS_REG\r\n");
		return 0;
	}

	oldReg1Value = ptrlis2dh12_register->lis2dh12_CTRL_REG1._Lis2dh12_CTRL_REG1;

	ptrlis2dh12_register->lis2dh12_CTRL_REG1.ODR0 = 0;
	ptrlis2dh12_register->lis2dh12_CTRL_REG1.ODR1 = 0;
	ptrlis2dh12_register->lis2dh12_CTRL_REG1.ODR2 = 0;

	lis2dh12_SendData(Lis2dh12_ADDR_CTRL_REG1, (uint8_t *)&ptrlis2dh12_register->lis2dh12_CTRL_REG1, UNCHECK_WRITTEN_DATA);

	return oldReg1Value;
}

/*********************************************************************
 * @param oldCtrlReg1Value, CTRL_REG1 value before calling lis2dh12_powerDownMode() func
 * @return Returns old CTRL_REG1 value
**********************************************************************/
uint8_t lis2dh12_wakeUpMode(Lis2dh12_register *ptrlis2dh12_register, uint8_t oldCtrlReg1Value)
{
	ptrlis2dh12_register->lis2dh12_CTRL_REG1._Lis2dh12_CTRL_REG1 = oldCtrlReg1Value;

	lis2dh12_SendData(Lis2dh12_ADDR_CTRL_REG1, (uint8_t *)&ptrlis2dh12_register->lis2dh12_CTRL_REG1, UNCHECK_WRITTEN_DATA);

	return 1;
}



/*********************************************************************
 * @param checkCntrl: read and compare the written data.
 * @return Returns non-zero value on successful all controls
**********************************************************************/
uint8_t lis2dh12_readReg_REFERENCE(Lis2dh12_register *ptrlis2dh12_register)
{
	if(!lis2dh12_ReadData(Lis2dh12_ADDR_STATUS_REG, &ptrlis2dh12_register->lis2dh12_REFERENCE.lis2dh12_reference)){
		//_myprintf("problem when write/read REFERENCE\r\n");
		return 0;
	}

	return 1;
}

/*********************************************************************
 * @param checkCntrl: read and compare the written data.
 * @return Returns non-zero value on successful all controls
**********************************************************************/
uint8_t lis2dh12_setReg_REFERENCE(Lis2dh12_register *ptrlis2dh12_register, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	/** Write REFERENCE *******************************/
	ptrlis2dh12_register->lis2dh12_REFERENCE.lis2dh12_reference = 0;			//  (Def:0)
	ret &= lis2dh12_SendData(Lis2dh12_ADDR_REFERENCE, (uint8_t *)&ptrlis2dh12_register->lis2dh12_REFERENCE.lis2dh12_reference, checkCntrl);

	return ret;
}


/*********************************************************************
 * @param mgSens: sensitivity value in mg multiplier.
 *  			   1 LSb = 16 mg @ FS = 2 g
 * @return Returns non-zero value on successful all controls
**********************************************************************/
uint8_t lis2dh12_setReg_INTxTHS(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrLis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	ptrlis2dh12_register->lis2dh12_INTxTHS.lis2dh12_INT1THS = ptrLis2dh12_config->mgSens;			//  (Def:0)
	ptrlis2dh12_register->lis2dh12_INTxTHS.lis2dh12_INT2THS = ptrLis2dh12_config->mgSens;			//  (Def:0)

	ret &= lis2dh12_SendData(Lis2dh12_ADDR_INT1_THS, (uint8_t *)&ptrlis2dh12_register->lis2dh12_INTxTHS.lis2dh12_INT1THS, checkCntrl);
	ret &= lis2dh12_SendData(Lis2dh12_ADDR_INT2_THS, (uint8_t *)&ptrlis2dh12_register->lis2dh12_INTxTHS.lis2dh12_INT2THS, checkCntrl);

	return ret;
}

/*********************************************************************
 * Duration time is measured in N/ODR, where N is the content of the duration register.
 * Not: Duration steps and maximum values depend on the ODR chosen.
 * @param checkCntrl: read and compare the written data.
 * @return Returns non-zero value on successful all controls
**********************************************************************/
uint8_t lis2dh12_setReg_INTxDURATION(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrLis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	/** Write INTx_DURATION *******************************/
	ptrlis2dh12_register->lis2dh12_INTxDURATION.lis2dh12_INT1DURATION = ptrLis2dh12_config->intDuration;			//  (Def:0)
	ptrlis2dh12_register->lis2dh12_INTxDURATION.lis2dh12_INT2DURATION = ptrLis2dh12_config->intDuration;			//  (Def:0)

	ret &= lis2dh12_SendData(Lis2dh12_ADDR_INT1_DURATION, (uint8_t *)&ptrlis2dh12_register->lis2dh12_INTxDURATION.lis2dh12_INT1DURATION, checkCntrl);
	ret &= lis2dh12_SendData(Lis2dh12_ADDR_INT2_DURATION, (uint8_t *)&ptrlis2dh12_register->lis2dh12_INTxDURATION.lis2dh12_INT2DURATION, checkCntrl);

	return ret;
}


/*********************************************************************
 * @param checkCntrl: read and compare the written data.
 * @return Returns non-zero value on successful all controls
**********************************************************************/
uint8_t lis2dh12_setReg_INT1CFG(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	/** Write set INT1_CFG *******************************/
	ptrlis2dh12_register->lis2dh12_INT1CFG.XLIE = 0;	// Enable interrupt generation on X low event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT1CFG.XHIE = ((ptrlis2dh12_config->intSource>>1)&0x01);	// Enable interrupt generation on X high event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT1CFG.YLIE = 0;	// Enable interrupt generation on Y low event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT1CFG.YHIE = ((ptrlis2dh12_config->intSource>>3)&0x01);	// Enable interrupt generation on Y high event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT1CFG.ZLIE = 0;	// Enable interrupt generation on Z low event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT1CFG.ZHIE = ((ptrlis2dh12_config->intSource>>5)&0x01);	// Enable interrupt generation on Z high event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT1CFG.DIR_6D = 0;	// 6-direction detection function enabled. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT1CFG.AOI = ((ptrlis2dh12_config->intSource>>7)&0x01);		// And/Or combination of interrupt events. (Def:0 "and")

	ret &= lis2dh12_SendData(Lis2dh12_ADDR_INT1_CFG, (uint8_t *)&ptrlis2dh12_register->lis2dh12_INT1CFG, checkCntrl);

	return ret;
}

/*********************************************************************
 * @param checkCntrl: read and compare the written data.
 * @return Returns non-zero value on successful all controls
**********************************************************************/
uint8_t lis2dh12_setReg_INT2CFG(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	/** Write set INT1_CFG *******************************/
	ptrlis2dh12_register->lis2dh12_INT2CFG.XLIE = 0;	// Enable interrupt generation on X low event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT2CFG.XHIE = ((ptrlis2dh12_config->intSource>>1)&0x01);	// Enable interrupt generation on X high event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT2CFG.YLIE = 0;	// Enable interrupt generation on Y low event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT2CFG.YHIE = ((ptrlis2dh12_config->intSource>>3)&0x01);	// Enable interrupt generation on Y high event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT2CFG.ZLIE = 0;	// Enable interrupt generation on Z low event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT2CFG.ZHIE = ((ptrlis2dh12_config->intSource>>5)&0x01);	// Enable interrupt generation on Z high event or on direction recognition. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT2CFG.DIR_6D = 0;	// 6-direction detection function enabled. (Def:0)
	ptrlis2dh12_register->lis2dh12_INT2CFG.AOI = ((ptrlis2dh12_config->intSource>>7)&0x01);		// And/Or combination of interrupt events. (Def:0 "and")

	ret &= lis2dh12_SendData(Lis2dh12_ADDR_INT2_CFG, (uint8_t *)&ptrlis2dh12_register->lis2dh12_INT2CFG, checkCntrl);

	return ret;
}

/*********************************************************************
 * @param mgSens: sensitivity value in mg multiplier.
 *  			   1 LSb = 16 mg @ FS = 2 g
 * @return Returns non-zero value on successful all controls
**********************************************************************/
uint8_t lis2dh12_setReg_ACTTHS(Lis2dh12_register *ptrlis2dh12_register,  Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	ptrlis2dh12_register->lis2dh12_ACT_THS.lis2dh12_ACT_THS = ptrlis2dh12_config->mgSens;			//  (Def:0)

	ret &= lis2dh12_SendData(Lis2dh12_ADDR_ACT_THS, (uint8_t *)&ptrlis2dh12_register->lis2dh12_ACT_THS.lis2dh12_ACT_THS, checkCntrl);

	return ret;
}

/*********************************************************************
 * @param mgSens: Sleep-to-wake, return-to-sleep duration.
 *  			  1 LSb = (8*1[LSb]+1)/ODR
 * @return Returns non-zero value on successful all controls
**********************************************************************/
uint8_t lis2dh12_setReg_ACTDUR(Lis2dh12_register *ptrlis2dh12_register,  Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl)
{
	uint8_t ret = 1;

	ptrlis2dh12_register->lis2dh12_ACT_DUR.lis2dh12_ACT_DUR = ptrlis2dh12_config->actDuration;

	ret &= lis2dh12_SendData(Lis2dh12_ADDR_ACT_DUR, (uint8_t *)&ptrlis2dh12_register->lis2dh12_ACT_DUR.lis2dh12_ACT_DUR, checkCntrl);

	return ret;
}

/*********************************************************************

 * Default value: 00010000 (0: pull-up connected to SDO/SA0 pin; 1: pull-up disconnected to SDO/SA0 pin)

 * @param EnbDsb: for Enable PullUp param EnbDsb should be 0.

 * @return Returns non-zero value if Lis2dh12 IC is valid.

**********************************************************************/

uint8_t set_SDOPullUp_EnbDsb(uint8_t EnbDsb, uint8_t checkCntrl)
{
    uint8_t ret = 1;
    uint8_t defVal = 0x10;

    defVal |= EnbDsb << BIT7;

	/** Write 90h to CTRL_REG1 (1Eh) **************************************/
	ret = lis2dh12_SendData(Lis2dh12_ADDR_CTRL_REG0, (uint8_t *)&defVal, checkCntrl);

	return ret;
}


/****************************************************************************
 * this function must be called at least once before starting operations with lis2.
 * the return value must be an address from the Lis2dh12_register structure type in order to avoid any loss of value.
 *
 * CTRL_REG1: enable x,y,z axis, set operation mode "Low-power mode (50 Hz)".
 * CTRL_REG2: disable "High-pass filter".
 * CTRL_REG3: no interrupt source select.
 * CTRL_REG4: scale selection is ±2g.
 * CTRL_REG5: latch interrupt request on INT1_SRC.
 * CTRL_REG6: disable interrupt activity.
 *
 * @param checkCntrl: read and compare the written data.
 * @return Returns address from the Lis2dh12_register structure type
******************************************************************************/
Lis2dh12_register *lis2dh12_initDefConfig(lis2dh12_devList devAddr, uint8_t checkCntrl)
{
	uint8_t ret = 1;
	uint8_t ntimes = 3;
	Lis2dh12_register *ptrlis2dh12_register = getLocalptr_Lis2dh12register(devAddr);
	Lis2dh12_config *ptrlis2dh12_config = getLocalptr_Lis2dh12config(devAddr);

	/* if not initial before */
	if((ptrlis2dh12_config->dataRate == 0) && (ptrlis2dh12_config->mgSens == Mg0)){
		ptrlis2dh12_config->dataRate = Hz25;				/* ODR:  Data rate configuration */
		ptrlis2dh12_config->mgSens = Mg64;					/* Mg64 */
		ptrlis2dh12_config->hpFilter = HP_DISABLE;
		ptrlis2dh12_config->intSource = INT_X|INT_Y|INT_Z;
		ptrlis2dh12_config->intDuration = 1;				/* Duration time is measured in N/ODR, where N is the content of the duration register */
		ptrlis2dh12_config->actDuration = 1;				/* Sleep-to-wake, return-to-sleep duration. 1 LSb = (8*1[LSb]+1)/ODR */
	}

	do{
		memset(ptrlis2dh12_register, 0x00, sizeof(Lis2dh12_register));
		/* pull-up disconnected to SDO/SA0 pin*/
		ret &= set_SDOPullUp_EnbDsb(1, checkCntrl);
		/* init lis2dh12 IC for "HP filter bypassed" mode */
		ret &= lis2dh12_setReg_CTRL_REG1(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);
		ret &= lis2dh12_setReg_CTRL_REG2(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);
		ret &= lis2dh12_setReg_CTRL_REG3(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);
		ret &= lis2dh12_setReg_CTRL_REG4(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);
		ret &= lis2dh12_setReg_CTRL_REG5(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);
		ret &= lis2dh12_setReg_CTRL_REG6(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);

		ret &= lis2dh12_readReg_REFERENCE(ptrlis2dh12_register);
		ret &= lis2dh12_setReg_INTxTHS(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);
		ret &= lis2dh12_setReg_INTxDURATION(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);
		ret &= lis2dh12_readReg_REFERENCE(ptrlis2dh12_register);
		ret &= lis2dh12_setReg_INT1CFG(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);
		ret &= lis2dh12_setReg_ACTTHS(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);
		ret &= lis2dh12_setReg_ACTDUR(ptrlis2dh12_register, ptrlis2dh12_config, checkCntrl);

		//delayMs(125);												// Power up, wait for 90ms for stable output
		for(int i=0; i < 1250; i++)
			I2C_BlockingDelay_us(100);

		ret &= lis2dh12_readReg_XYZ(ptrlis2dh12_register);			// XDA, YDA, ZDA are individually reset to 0 when the respective higher part of the data is read .
		//delayMs(1);
		I2C_BlockingDelay_us(1000);

		if(ret)
			break;
	}while(ntimes--);

	if(ntimes <= 0){
//		_myprintf("can not configured Lis2dh12\r\n");
		return 0;
	}

	if(ret)
		return ptrlis2dh12_register;
	else
		return NULL;
}

/*********************************************************************
 * @param :
 * @return Returns non-zero value if Lis2dh12 IC is valid.
**********************************************************************/
uint8_t set_AccINT1EnbDsb(Lis2dh12_INTSource enbXYZ, lis2dh12_devList dev, uint8_t checkCntrl)
{
	Lis2dh12_register *ptrlis2dh12_register = getLocalptr_Lis2dh12register(dev);
	Lis2dh12_config *ptrlis2dh12_config = getLocalptr_Lis2dh12config(dev);
	uint8_t ret = 0;

	ptrlis2dh12_config->intSource = enbXYZ;

	ret &= lis2dh12_setReg_INT1CFG(ptrlis2dh12_register, ptrlis2dh12_config, UNCHECK_WRITTEN_DATA);

	if(enbXYZ != INT_DSb){
		ptrlis2dh12_config->int1act = ACT_I1_IA1;		/*interrupt activity set forced ACT_I1_IA1 */
		ret &= lis2dh12_setReg_CTRL_REG3(ptrlis2dh12_register, ptrlis2dh12_config, UNCHECK_WRITTEN_DATA);

		ret &= lis2dh12_SendData(Lis2dh12_ADDR_CTRL_REG3, (uint8_t *)&ptrlis2dh12_register->lis2dh12_CTRL_REG3, DISABLE);
	}

	return 1;
}






#endif /* PROJECTS_BLE_ANCHOR_LIS2DH12_MAIN_H_ */
