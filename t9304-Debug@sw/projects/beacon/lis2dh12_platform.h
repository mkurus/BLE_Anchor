/*=============================================
 Name        : lis2dh12_platform.h
 Author      : rusen Bilge
 Version     : V1.00
 Description :
================================================*/

#ifndef LIS2DH12_PLATFORM_H_
#define LIS2DH12_PLATFORM_H_

//#include "chip.h"
//#include "lpc_acc.h"

/*-----------------------------------------*/
/*-----------------------------------------*/
typedef enum WakeUpReason{VIAERR, VIAEXTINT, VIAWDT}wakeUpReason;
/*-----------------------------------------*/

typedef struct{
	uint8_t settingsOK;
	uint32_t sleepTime;					/* in mins */
	uint32_t alarmTime;					/* in mins */
	wakeUpReason whyWakeUp;
}DefProjectSettings;


/*---------------- default settings ----------------*/
#define DEF_MOTION_DETEC_MGval					Mg48
#define DEF_MOTION_DETEC_ALARM_TIME				60		//sec
#define DEF_MOTION_DETEC_SLEEP_TIME				1		//sec

#define DEF_MOTIONLESS_DETEC_MGval				Mg32
#define DEF_MOTIONLESS_DETEC_ALARM_TIME			60		//sec
#define DEF_MOTIONLESS_DETEC_SLEEP_TIME			1		//sec

#define DEF_ANGLE_METER_MGval					Mg64
#define DEF_ANGLE_METER_ALARM_TIME				60		//sec
#define DEF_ANGLE_METER_SLEEP_TIME				1		//sec

#define DEF_FREE_FALL_DETEC_MGval				Mg352

/*--------------------------------------------------*/
#define EXTINT_MIN_PERIOD_MSEC					5		//msec
/*--------------------------------------------------*/

/*---------------- projecs  ------------------------*/
#define MOTION_DETECTION			0
#define MOTIONLESS_DETECTION		1
#define ANGLE_METER					0
#define FREE_FALL_DETECTION			0

#define	ALARM_LED					1
#define	ALARM_BUZZER				0

#if (MOTION_DETECTION+MOTIONLESS_DETECTION+ANGLE_METER+FREE_FALL_DETECTION!= 1)
	#error	"DEVICE CONFIG ERROR!!!!"
#endif

/*----------------------- Function Prototypes ------------------*/
void deviceSettings(DefProjectSettings *ptrdefProjectSettings);
wakeUpReason setWakeupStatus(void);

#endif /* LIS2DH12_LIS2DH12_PLATFORM_H_ */
