/*=============================================
 Name        : lis2dh12.c
 Author      : rusen Bilge
 Version     : V1.00
 Description :
================================================*/


#ifndef LIS2DH12_H_
#define LIS2DH12_H_

#include "lis2dh12_platform.h"


/****************************************************/
/** lis2dh12 devices 								*/
/****************************************************/
typedef enum devList{DEV0_LIS, DEV1_LIS}lis2dh12_devList;

/****************************************************/
/** event intterrupt call back structure 			*/
/****************************************************/
enum eventINT{INT1, INT2};

extern void eventINT1CallbackFunc(void);
extern void eventINT2CallbackFunc(void);

typedef struct _tagEvent{
	char getINTxEvent;
	void(*callBackfunc)(void);
}Event;

extern Event event[2];

/****************************************************/
/**  	*/
/****************************************************/
#define CHECK_WRITTEN_DATA		1
#define UNCHECK_WRITTEN_DATA	0

/****************************************************/
/** IC addr(7bit) and read/write command(1bit)	 	*/
/****************************************************/
//#define CMD_READ			0x31
//#define CMD_WRITE			0x30

#define Lis2dh12_DEV_ADDR_msb7BIT		0x18
#define Lis2dh12_DEV_ADDR_lsb1BIT		0		// depent on lis2dh12 pin3 (SA0) gnd:0, vdd:1

/*********************************************/
/** default registers value			 		 */
/*********************************************/
#define Lis2dh12_DEF_VALUE_WHO_AM_I		0x33


/*********************************************/
/** registers address				 		 */
/*********************************************/
/****** info registers address ***************/
#define Lis2dh12_ADDR_STATUS_REG_AUX	0x07
#define Lis2dh12_ADDR_OUT_TEMP_L		0x0C
#define Lis2dh12_ADDR_OUT_TEMP_H		0x0D
#define Lis2dh12_ADDR_REG_WHO_AM_I		0x0F

/****** control registers address ************/
#define Lis2dh12_ADDR_CTRL_REG0			0x1E
#define Lis2dh12_ADDR_TEMP_CFG_REG		0x1F
#define Lis2dh12_ADDR_CTRL_REG1			0x20
#define Lis2dh12_ADDR_CTRL_REG2			0x21
#define Lis2dh12_ADDR_CTRL_REG3			0x22
#define Lis2dh12_ADDR_CTRL_REG4			0x23
#define Lis2dh12_ADDR_CTRL_REG5			0x24
#define Lis2dh12_ADDR_CTRL_REG6			0x25

/****** xxxxxx registers address ************/
#define Lis2dh12_ADDR_REFERENCE			0x26
#define Lis2dh12_ADDR_STATUS_REG		0x27

/****** xxxxxx registers address ************/
#define Lis2dh12_ADDR_OUT_X_L			0x28
#define Lis2dh12_ADDR_OUT_X_H			0x29
#define Lis2dh12_ADDR_OUT_Y_L			0x2A
#define Lis2dh12_ADDR_OUT_Y_H			0x2B
#define Lis2dh12_ADDR_OUT_Z_L			0x2C
#define Lis2dh12_ADDR_OUT_Z_H			0x2D

/****** xxxxxx registers address ************/
#define Lis2dh12_ADDR_FIFO_CTRL_REG		0x2E
#define Lis2dh12_ADDR_FIFO_SRC_REG		0x2F

/****** xxxxxx registers address ************/
#define Lis2dh12_ADDR_INT1_CFG			0x30
#define Lis2dh12_ADDR_INT1_SRC			0x31
#define Lis2dh12_ADDR_INT1_THS			0x32
#define Lis2dh12_ADDR_INT1_DURATION		0x33
#define Lis2dh12_ADDR_INT2_CFG			0x34
#define Lis2dh12_ADDR_INT2_SRC			0x35
#define Lis2dh12_ADDR_INT2_THS			0x36
#define Lis2dh12_ADDR_INT2_DURATION		0x37

/****** xxxxxx registers address ************/
#define Lis2dh12_ADDR_CLICK_CFG			0x38
#define Lis2dh12_ADDR_CLICK_SRC			0x39
#define Lis2dh12_ADDR_CLICK_THS			0x3A

/****** xxxxxx registers address ************/
#define Lis2dh12_ADDR_TIME_LIMIT		0x3B
#define Lis2dh12_ADDR_TIME_LATENCY		0x3C
#define Lis2dh12_ADDR_TIME_WINDOW		0x3D
#define Lis2dh12_ADDR_ACT_THS			0x3E
#define Lis2dh12_ADDR_ACT_DUR			0x3F
/*********************************************/
/*********************************************/
typedef enum INTSource{INT_DSb=0, INT_X =2, INT_Y = 8, INT_Z = 32, INTs_AND = 128}Lis2dh12_INTSource;
typedef enum DataRate{Hz1=1, Hz10 , Hz25, Hz50, Hz100, Hz200, Hz400, Hz1620, Hz5376}Lis2dh12_DataRate;
typedef enum HPFilter{HP_DISABLE = 0, HP_ENABLE = 1}Lis2dh12_HPFilter;
typedef enum INT1activity{ACT_I1_ZYXDA=16, ACT_I1_IA2=32, ACT_I1_IA1=64 }Lis2dh12_INT1activity;
typedef enum FullScaleSelect{FS2g=0, FS4g, FS8g, FS16g}Lis2dh12_FullScaleSelect;
typedef enum Sensivity{Mg0=0, Mg16=1, Mg32=2, Mg48=3, Mg64=4, Mg128=8, Mg256=16,
										Mg352=22, Mg416=26, Mg480=30, Mg644=34  }Lis2dh12_Sensivity;



typedef struct _tagLis2dh12_Lis2dh12_CONFIG{
	Lis2dh12_DataRate dataRate;					// Data rate selection
	Lis2dh12_Sensivity mgSens;
	Lis2dh12_HPFilter hpFilter;
	Lis2dh12_INTSource intSource;
	Lis2dh12_INT1activity int1act;
	uint8_t intDuration;
	uint8_t actDuration;
}Lis2dh12_config;

extern Lis2dh12_config lis2dh12_config[2];

/*********************************************/
/** 			 		 */
/** 			 		 */
/*********************************************/
typedef union _tagLis2dh12_CTRL_REG1{
	struct {
		uint8_t Xen:1;				// X-axis enable. Default value: 1(enb)
		uint8_t Yen:1;				// Y-axis enable. Default value: 1(enb)
		uint8_t Zen:1;				// Z-axis enable. Default value: 1(enb)
		uint8_t LPen:1;				// Low-power mode enable. Def:0	 (0: high-resolution /normal mode, 1: low-power mode)
		uint8_t ODR0:1;				// 0000: Power-down mode
		uint8_t ODR1:1;				// 0001: HR / Normal / Low-power mode (1 Hz)	... ...
		uint8_t ODR2:1;				// 0010: HR / Normal / Low-power mode (50 Hz)	... ...
		uint8_t ODR3:1;				// 1000: Low-power mode
	};
	uint8_t _Lis2dh12_CTRL_REG1;
}Lis2dh12_CTRL_REG1;


typedef union _tagLis2dh12_CTRL_REG2{
	struct {
		uint8_t HP_IA1:1;			// High-pass filter enabled for AOI function on Interrupt 1 (0: filter bypassed; 1: filter enabled)
		uint8_t HP_IA2:1;			// High-pass filter enabled for AOI function on Interrupt 2 (0: filter bypassed; 1: filter enabled)
		uint8_t HPCLICK:1;			// High-pass filter enabled for CLICK function. (0: filter bypassed; 1: filter enabled)
		uint8_t FDS:1;				// Filtered data selection. Default value: 0 (0: internal filter bypassed; 1: data from internal filter sent to output register and FIFO)
		uint8_t HPCF1:1;			// High-pass filter cutoff frequency selection
		uint8_t HPCF2:1;			//
		uint8_t HPM0:1;				// 00: Normal mode 01: Reference signal for filtering	(Default value: 00)
		uint8_t HPM1:1;				// 10: Normal mode 11: Autoreset on interrupt event
	};
	uint8_t _Lis2dh12_CTRL_REG2;
}Lis2dh12_CTRL_REG2;


typedef union _tagLis2dh12_CTRL_REG3{
	struct {
		uint8_t reserved:1;			// reserved
		uint8_t I1_OVERRUN:1;		// FIFO overrun interrupt on INT1 pin. (Def:0) (0: disable; 1: enable)
		uint8_t I1_WTM:1;			// FIFO watermark interrupt on INT1 pin. (Def:0) (0: disable; 1: enable)
		uint8_t VAL0:1;				// must be "0"
		uint8_t I1_ZYXDA:1;			// ZYXDA interrupt on INT1 pin. (Def:0) (0: disable; 1: enable)
		uint8_t I1_IA2:1;			// IA2 interrupt on INT1 pin. (Def:0) (0: disable; 1: enable)
		uint8_t I1_IA1:1;			// IA1 interrupt on INT1 pin. (Def:0) (0: disable; 1: enable)
		uint8_t I1_CLICK:1;			// CLICK interrupt on INT1 pin. (Def:0) (0: disable; 1: enable)
	};
	uint8_t _Lis2dh12_CTRL_REG3;
}Lis2dh12_CTRL_REG3;


typedef union _tagLis2dh12_CTRL_REG4{
	struct {
		uint8_t SIM:1;				// SPI serial interface mode selection. (Def:0) (0: 4-wire interface; 1: 3-wire interface).
		uint8_t ST0:1;				// Self-test enable. (Def:00)
		uint8_t ST1:1;				// (00: self-test disabled, 01:Self test 0, 10:Self test 1)
		uint8_t HR:1;				// Operating mode selection (1: High-resolution, normal mode, low-power mode)
		uint8_t FS0:1;				// Full-scale selection. (Def:00)
		uint8_t FS1:1;				// ( 00: ±2g;  01: ±4g;  10: ±8g;  11: ±16g )
		uint8_t BLE:1;				// Big/Little Endian data selection. (Def:0)
		uint8_t BDU:1;				// Block data update. (Def:0)  (0: continuous update; 1: output registers not updated until MSB and LSB have been read)
	};
	uint8_t _Lis2dh12_CTRL_REG4;
}Lis2dh12_CTRL_REG4;


typedef union _tagLis2dh12_CTRL_REG5{
	struct {
		uint8_t D4D_INT2:1;			// 4D enable: 4D detection is enabled on INT2 pin when 6D bit on INT2_CFG (34h) is set to 1.
		uint8_t LIR_INT2:1;			// Latch interrupt request on INT2_SRC (35h) register, with INT2_SRC (35h) register cleared by reading INT2_SRC (35h) itself. Def: 0.
		uint8_t D4D_INT1:1;			// 4D enable: 4D detection is enabled on INT1 pin when 6D bit on INT1_CFG (30h) is set to 1.
		uint8_t LIR_INT1:1;			// Latch interrupt request on INT1_SRC (31h), with INT1_SRC (31h) register cleared by reading INT1_SRC (31h) itself. Def: 0.
		uint8_t rev1:1;				// reserved
		uint8_t rev2:1;				// reserved
		uint8_t FIFO_EN:1;			// FIFO enable. (Def:0) (0: FIFO disabled; 1: FIFO enabled)
		uint8_t BOOT:1;				// Reboot memory content. (Def:0) (0: normal mode; 1: reboot memory content)
	};
	uint8_t _Lis2dh12_CTRL_REG5;
}Lis2dh12_CTRL_REG5;


typedef union _tagLis2dh12_CTRL_REG6{
	struct {
		uint8_t rev1:1;				// reserved
		uint8_t INT_POLARITY:1;		// INT1 and INT2 pin polarity. (Def:0) (0: active-high; 1: active-low)
		uint8_t rev2:1;				// reserved
		uint8_t I2_ACT:1;			// Enable activity interrupt on INT2 pin. (Def:0) (0: disabled; 1: enabled)
		uint8_t I2_BOOT:1;			// Enable boot on INT2 pin. (Def:0) (0: disabled; 1: enabled)
		uint8_t I2_IA2:1;			// Enable interrupt 2 function on INT2 pin. (Def:0) (0: disabled; 1: enabled)
		uint8_t I2_IA1:1;			// Enable interrupt 1 function on INT2 pin. (Def:0) (0: disabled; 1: enabled)
		uint8_t I2_CLICK:1;			// Click interrupt on INT2 pin. (Def:0) (0: disabled; 1: enabled)
	};
	uint8_t _Lis2dh12_CTRL_REG6;
}Lis2dh12_CTRL_REG6;


typedef union _tagLis2dh12_REFERENCE{
		uint8_t	lis2dh12_reference;			// Reference value for interrupt generation. (Def:0)
}Lis2dh12_REFERENCE;


/*
 * THS[6:0]
 * 1 LSb =  16mg @ FS = 2g
 * 1 LSb =  32mg @ FS = 4g
 * 1 LSb =  62mg @ FS = 8g
 * 1 LSb = 186mg @ FS = 16g
 */
typedef union _tagLis2dh12_INTxTHS{
	struct {
		uint8_t	lis2dh12_INT1THS;			// Interrupt 1 threshold. (Def:000 0000)
		uint8_t	lis2dh12_INT2THS;			// Interrupt 2 threshold. (Def:000 0000)
	};
	uint16_t _Lis2dh12_INTxTHS;
}Lis2dh12_INTxTHS;


typedef union _tagLis2dh12_INTxDURATION{
	struct {								// Duration time is measured in N/ODR, where N is the content of the duration register.
		uint8_t	lis2dh12_INT1DURATION;		// Duration value. 1LSb = 1/ODR (Def:000 0000)
		uint8_t	lis2dh12_INT2DURATION;		// Duration value. 1LSb = 1/ODR (Def:000 0000)
	};
	uint16_t _Lis2dh12_INTxDURATION;
}Lis2dh12_INTxDURATION;


typedef	union _tagLis2dh12_INT1CFG{
	struct {
		uint8_t XLIE:1;				// Enable interrupt generation on X low event or on direction recognition. (Def:0)
		uint8_t XHIE:1;				// Enable interrupt generation on X high event or on direction recognition. (Def:0)
		uint8_t YLIE:1;				// Enable interrupt generation on Y low event or on direction recognition. (Def:0)
		uint8_t YHIE:1;				// Enable interrupt generation on Y high event or on direction recognition. (Def:0)
		uint8_t ZLIE:1;				// Enable interrupt generation on Z low event or on direction recognition. (Def:0)
		uint8_t ZHIE:1;				// Enable interrupt generation on Z high event or on direction recognition. (Def:0)
		uint8_t DIR_6D:1;			// 6-direction detection function enabled. (Def:0)
		uint8_t AOI:1;				// And/Or combination of interrupt events. (Def:0)
	};
	uint8_t _Lis2dh12_INT1CFG;
}Lis2dh12_INT1CFG;

typedef	union _tagLis2dh12_INT2CFG{
	struct {
		uint8_t XLIE:1;				// Enable interrupt generation on X low event or on direction recognition. (Def:0)
		uint8_t XHIE:1;				// Enable interrupt generation on X high event or on direction recognition. (Def:0)
		uint8_t YLIE:1;				// Enable interrupt generation on Y low event or on direction recognition. (Def:0)
		uint8_t YHIE:1;				// Enable interrupt generation on Y high event or on direction recognition. (Def:0)
		uint8_t ZLIE:1;				// Enable interrupt generation on Z low event or on direction recognition. (Def:0)
		uint8_t ZHIE:1;				// Enable interrupt generation on Z high event or on direction recognition. (Def:0)
		uint8_t DIR_6D:1;			// 6-direction detection function enabled. (Def:0)
		uint8_t AOI:1;				// And/Or combination of interrupt events. (Def:0)
	};
	uint8_t _Lis2dh12_INT2CFG;
}Lis2dh12_INT2CFG;

typedef union _tagLis2dh12_INT1SRC{
	struct {
		uint8_t XL:1;				// X low.  Default value: 0 (0: no interrupt, 1: X low event has occurred)
		uint8_t XH:1;				// X high. Default value: 0 (0: no interrupt, 1: X high event has occurred)
		uint8_t YL:1;				// Y low.  Default value: 0 (0: no interrupt, 1: Y low event has occurred)
		uint8_t YH:1;				// Y high. Default value: 0 (0: no interrupt, 1: Y high event has occurred)
		uint8_t ZL:1;				// Z low.  Default value: 0 (0: no interrupt; 1: Z low event has occurred)
		uint8_t ZH:1;				// Z high. Default value: 0 (0: no interrupt, 1: Z high event has occurred)
		uint8_t IA:1;				// Interrupt active. Default value: 0	(0: no interrupt has been generated; 1: one or more interrupts have been generated)
		uint8_t val0:1;				//
	};
	uint8_t _Lis2dh12_INT1SRC;
}Lis2dh12_INT1SRC;

typedef union _tagLis2dh12_INT2SRC{
	struct {
		uint8_t XL:1;				// X low.  Default value: 0 (0: no interrupt, 1: X low event has occurred)
		uint8_t XH:1;				// X high. Default value: 0 (0: no interrupt, 1: X high event has occurred)
		uint8_t YL:1;				// Y low.  Default value: 0 (0: no interrupt, 1: Y low event has occurred)
		uint8_t YH:1;				// Y high. Default value: 0 (0: no interrupt, 1: Y high event has occurred)
		uint8_t ZL:1;				// Z low.  Default value: 0 (0: no interrupt; 1: Z low event has occurred)
		uint8_t ZH:1;				// Z high. Default value: 0 (0: no interrupt, 1: Z high event has occurred)
		uint8_t IA:1;				// Interrupt active. Default value: 0	(0: no interrupt has been generated; 1: one or more interrupts have been generated)
		uint8_t val0:1;				//
	};
	uint8_t _Lis2dh12_INT2SRC;
}Lis2dh12_INT2SRC;


typedef union _tagLis2dh12_STATUS_REG{
	struct {
		uint8_t XDA:1;				// X-axis new data available. (Def:0) (0: new data not yet available; 1: new data  available)
		uint8_t YDA:1;				// Y-axis new data available. (Def:0) (0: new data not yet available; 1: new data  available)
		uint8_t ZDA:1;				// Z-axis new data available. (Def:0) (0: new data not yet available; 1: new data  available)
		uint8_t ZYXDA:1;			// ZYXDA-axis new data available. (Def:0) (0: new data not yet available; 1: new data  available)
		uint8_t XOR:1;				// X-axis data overrun. (Def:0) (0: no overrun has occurred; 1: new data overwritten the previous data)
		uint8_t YOR:1;				// Y-axis data overrun. (Def:0) (0: no overrun has occurred; 1: new data overwritten the previous data)
		uint8_t ZOR:1;				// Z-axis data overrun. (Def:0) (0: no overrun has occurred; 1: new data overwritten the previous data)
		uint8_t ZYXOR:1;			// X, Y and Z-axis data overrun (Def:0) (0: no overrun has occurred; 1: new data overwritten the previous data)
	};
	uint8_t _Lis2dh12_STATUS_REG;
}Lis2dh12_STATUS_REG;


/* Acth[6:0] *************
 * 1 LSb = 16 mg @ FS = 2 g
 * 1 LSb = 32 mg @ FS = 4 g
 * 1 LSb = 62 mg @ FS = 8 g
 * 1 LSb = 186 mg @ FS = 16 g
 */
typedef struct _tagLis2dh12_ACT_THS{
		uint8_t	lis2dh12_ACT_THS;			//Sleep-to-wake, return-to-sleep activation threshold in low-power mode
}Lis2dh12_ACT_THS;

typedef struct _tagLis2dh12_ACT_DUR{
		uint8_t	lis2dh12_ACT_DUR;			//Sleep-to-wake, return-to-sleep activation threshold in low-power mode
}Lis2dh12_ACT_DUR;


typedef union _tagLis2dh12_OUT_X{
	struct {
		int8_t OUT_X_L;			// X-axis acceleration data.
		int8_t OUT_X_H;
	};
	uint16_t _Lis2dh12_OUT_X;
}Lis2dh12_OUT_X;

typedef union _tagLis2dh12_OUT_Y{
	struct {
		int8_t OUT_Y_L;			// Y-axis acceleration data.
		int8_t OUT_Y_H;
	};
	uint16_t _Lis2dh12_OUT_Y;
}Lis2dh12_OUT_Y;

typedef union _tagLis2dh12_OUT_Z{
	struct {
		int8_t OUT_Z_L;			// Z-axis acceleration data.
		int8_t OUT_Z_H;
	};
	uint16_t _Lis2dh12_OUT_Z;
}Lis2dh12_OUT_Z;



typedef struct _tagLis2dh12_register{
	Lis2dh12_CTRL_REG1		lis2dh12_CTRL_REG1;
	Lis2dh12_CTRL_REG2		lis2dh12_CTRL_REG2;
	Lis2dh12_CTRL_REG3		lis2dh12_CTRL_REG3;
	Lis2dh12_CTRL_REG4		lis2dh12_CTRL_REG4;
	Lis2dh12_CTRL_REG5		lis2dh12_CTRL_REG5;
	Lis2dh12_CTRL_REG6		lis2dh12_CTRL_REG6;
	Lis2dh12_REFERENCE		lis2dh12_REFERENCE;
	Lis2dh12_INTxTHS		lis2dh12_INTxTHS;
	Lis2dh12_ACT_THS		lis2dh12_ACT_THS;
	Lis2dh12_ACT_DUR		lis2dh12_ACT_DUR;
	Lis2dh12_INTxDURATION	lis2dh12_INTxDURATION;
	Lis2dh12_INT1CFG		lis2dh12_INT1CFG;
	Lis2dh12_INT2CFG		lis2dh12_INT2CFG;
	Lis2dh12_STATUS_REG		lis2dh12_STATUS_REG;
	Lis2dh12_INT1SRC		lis2dh12_INT1SRC;
	Lis2dh12_INT2SRC		lis2dh12_INT2SRC;
	Lis2dh12_OUT_X			lis2dh12_OUT_X;
	Lis2dh12_OUT_Y			lis2dh12_OUT_Y;
	Lis2dh12_OUT_Z			lis2dh12_OUT_Z;
}Lis2dh12_register;

extern Lis2dh12_register lis2dh12_register[2];

/* event func declaration */
void eventINT1(void);
void eventINT2(void);

typedef struct _tagCallBackEvent{
	char event;
	void(*callbackINT1EventFunc)(void);
}callBackEvent;



typedef struct _tagLis2dh12_output{
	int16_t		xAxisMgVal;
	int16_t		yAxisMgVal;
	int16_t		zAxisMgVal;
	int16_t		xAxisAngleVal;
	int16_t		yAxisAngleVal;
	int16_t		zAxisAngleVal;
}Lis2dh12_output;

extern Lis2dh12_output lis2dh12_output[2];


#define LIS2DH12_FROM_FS_2g_HR_TO_mg(lsb)  (float)((int16_t)lsb>>4)* 1.0f
#define LIS2DH12_FROM_FS_4g_HR_TO_mg(lsb)  (float)((int16_t)lsb>>4)* 2.0f
#define LIS2DH12_FROM_FS_8g_HR_TO_mg(lsb)  (float)((int16_t)lsb>>4)* 4.0f
#define LIS2DH12_FROM_FS_16g_HR_TO_mg(lsb) (float)((int16_t)lsb>>4)*12.0f
#define LIS2DH12_FROM_LSB_TO_degC_HR(lsb) (float)((int16_t)lsb>>6)/4.0f+25.0f

#define LIS2DH12_FROM_FS_2g_NM_TO_mg(lsb)  (float)((int16_t)lsb>>6)*  4.0f
#define LIS2DH12_FROM_FS_4g_NM_TO_mg(lsb)  (float)((int16_t)lsb>>6)*  8.0f
#define LIS2DH12_FROM_FS_8g_NM_TO_mg(lsb)  (float)((int16_t)lsb>>6)* 16.0f
#define LIS2DH12_FROM_FS_16g_NM_TO_mg(lsb) (float)((int16_t)lsb>>6)* 48.0f
#define LIS2DH12_FROM_LSB_TO_degC_NM(lsb) (float)((int16_t)lsb>>6)/4.0f+25.0f

#define LIS2DH12_FROM_FS_2g_LP_TO_mg(lsb)  (float)((int16_t)lsb>>8)*16.0f
#define LIS2DH12_FROM_FS_4g_LP_TO_mg(lsb)  (float)((int16_t)lsb>>8)*32.0f
#define LIS2DH12_FROM_FS_8g_LP_TO_mg(lsb)  (float)((int16_t)lsb>>8)*64.0f
#define LIS2DH12_FROM_FS_16g_LP_TO_mg(lsb) (float)((int16_t)lsb>>8)*192.0f
#define LIS2DH12_FROM_LSB_TO_degC_LP(lsb) (float)((int16_t)lsb>>8)*1.0f + 25.0f


/****************************************************/
/** functions prototype				 				*/
/****************************************************/
uint8_t lis2dh12_SendData(uint8_t addrReg, uint8_t *ptrRegister, uint8_t checkCntrl);
uint8_t lis2dh12_ReadData(uint8_t regAddr, uint8_t *rxBuffPtr);
uint8_t lis2dh12_I2C_Read(uint8_t regAddr, uint8_t *rxBuffPtr);
uint8_t lis2dh12_I2C_ReadBlock(uint8_t regAddr, uint8_t *rxBuffPtr);
uint8_t lis2dh12_I2C_Send(uint8_t regAddr, uint8_t *txBuffPtr);
uint8_t lis2dh12_I2C_SendBlock(uint8_t regAddr, uint8_t *txBuffPtr);
void lis2dh12_I2C_Stop();

/** init func **************************************/
uint8_t checkIC_Lis2dh12(lis2dh12_devList dev);
Lis2dh12_register *lis2dh12_initDefConfig(lis2dh12_devList devAddr, uint8_t checkCntrl);
Lis2dh12_register *getLocalptr_Lis2dh12register(lis2dh12_devList dev);
Lis2dh12_output *getLocalptr_Lis2dh12Output(lis2dh12_devList dev);
Lis2dh12_config *getLocalptr_Lis2dh12config(lis2dh12_devList dev);


/** scale convert func **************************************/
Lis2dh12_output *convert_Scale(Lis2dh12_register *ptrlis2dh12_register);

/** Read Register function **************************/
uint8_t lis2dh12_readReg_STATUS(Lis2dh12_register *ptrlis2dh12_register);
uint8_t lis2dh12_readReg_XYZ(Lis2dh12_register *ptrlis2dh12_register);
uint8_t lis2dh12_readReg_REFERENCE(Lis2dh12_register *ptrlis2dh12_register);
uint8_t lis2dh12_readReg_INT1SRC(Lis2dh12_register * ptrlis2dh12_register);

/** Set Register function **************************/
uint8_t lis2dh12_setReg_CTRL_REG1(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_CTRL_REG2(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_CTRL_REG3(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_CTRL_REG4(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_CTRL_REG5(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_CTRL_REG6(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl);

uint8_t lis2dh12_setReg_REFERENCE(Lis2dh12_register *ptrlis2dh12_register, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_INTxTHS(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrLis2dh12_config, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_INTxDURATION(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrLis2dh12_config, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_INT1CFG(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_INT2CFG(Lis2dh12_register *ptrlis2dh12_register, Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_ACTTHS(Lis2dh12_register *ptrlis2dh12_register,  Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl);
uint8_t lis2dh12_setReg_ACTDUR(Lis2dh12_register *ptrlis2dh12_register,  Lis2dh12_config *ptrlis2dh12_config, uint8_t checkCntrl);

/** power management function **************************/
uint8_t lis2dh12_powerDownMode(Lis2dh12_register *ptrlis2dh12_register);
uint8_t lis2dh12_wakeUpMode(Lis2dh12_register *ptrlis2dh12_register, uint8_t oldCtrlReg1Value);


/** APIs function **********************************/
uint8_t testMainApplication(lis2dh12_devList dev);
uint8_t set_AccINT1EnbDsb(Lis2dh12_INTSource enbXYZ, lis2dh12_devList dev, uint8_t checkCntrl);
uint8_t set_HPfilterEnbDsb(Lis2dh12_HPFilter enbDsb, lis2dh12_devList dev, uint8_t checkCntrl);
uint8_t set_AccDataRate(Lis2dh12_DataRate datarate, lis2dh12_devList dev, uint8_t checkCntrl);

uint8_t print_XYZoutputs(Lis2dh12_output *ptrlis2dh12_output, uint8_t rawEnb);
uint8_t setIntForGetOneValue(Lis2dh12_INTSource enbXYZ, Lis2dh12_INT1activity act, lis2dh12_devList dev);
uint8_t getINT1Mutex(void);
uint8_t freeINT1Mutex(void);
//-------------------------------------
#endif /* LIS2DH12_H_ */
