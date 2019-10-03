#include <types.h>
#include <macros.h>
#include <em_qpn.h>
#include <bsp.h>
#include <gpio.h>
#include <watchdog.h>
// Standard Includes
#include <string.h>
#include <timer_hal.h>
#include <t9304_uni_tim.h>
#include <timer.h>
#include <pml.h>
#include <i2c.h>
#include <i2c_hal.h>
#include "i2c_main.h"
#include <uart.h>
#include <printf.h>
#include "beacon.h"
#include "signals.h"
#include "lis2dh12_platform.h"
#include "lis2dh12.h"

#include "anchor_bsp.h"
#include "serial_task.h"

#define      UART_RX_COUNT_ON_LORA_RDY               ( 1u )
// Device ID -> Server tarafýnda paketin ne paketi olduðunu anlamasýna yarayacak deðer
#define      LORA_TRANSPORT_HEADER_BYTE			      0xAA

// Global variables.
Serial_t gSerialTask;
SECTION_NONPERSISTENT QEvt gSerialEventQueue[20];
char uartRxChar;

/**
 * Function declarations
 */
static QState Serial_Init(Serial_t *me);
static QState Serial_Idle_State(Serial_t *me);
static QState Serial_Xmit_State(Serial_t *me);
static QState Serial_Wait_Lora_WakeUp_State(Serial_t *me);
/*
 * UART callbacks
 */
void BLEAnchor_UartCallback(Driver_Status_t status, void *pUserData);
void BLEAnchor_UartTxCallback(Driver_Status_t status, void *pUserData);

/**
 * External variables
 */
extern int8_t lpcWakeUpTimerId;
extern int8_t wdtResetTimer;
extern Beacon_t gBeaconTask;
extern char lpcTxBuf[BLE_PAYLOAD_SIZE];
extern uint16_t lpcDataLen;
/**
 *
 */
void Serial_Constructor(Serial_t *me)
{
    // Tell QP-nano what the initial state is.
    QActive_ctor(&me->super, Q_STATE_CAST(Serial_Init));
}
/**
 *
 */

QState Serial_Init(Serial_t *me)
{
     return Q_TRAN(Serial_Idle_State);
}
/**
 *
 */
QState Serial_Idle_State(Serial_t *me)
{
	QState status;


	switch( Q_SIG(me) ) {

		case SERIAL_TASK_WAKEUP_LORA_SIG:
		//	printf("SERIAL_TASK_WAKEUP_LORA_SIG\r\n");
			// disable sleep mode
			PML_AutomaticSleepModeDisable(PML_AUTO_SLEEP_DISABLE_HCI, true);
	        // set timer to wait LPC wake-up
		    GPIO_SetHigh( GPIO_WAKEUP_NXP );
		    Timer_Restart( lpcWakeUpTimerId );
			//set UART callback to wait LPC ready character
			UART_ReceiveDataWithCallback( &uartRxChar, UART_RX_COUNT_ON_LORA_RDY, BLEAnchor_UartCallback, NULL );
			status = Q_TRAN(Serial_Wait_Lora_WakeUp_State);
			break;

		default:
			// Let the super class handle this.
			status = Q_SUPER(QHsm_top);
			break;
	}
	return status;
}
/**
 *
 */
QState Serial_Wait_Lora_WakeUp_State(Serial_t *me)
{
	QState status;


		switch( Q_SIG(me) ) {

		 case SERIAL_TASK_LORA_READY_SIG:
		 case SERIAL_TASK_LPC_WAKEUP_TIMEOUT_SIG:
		//	printf("LORA_READY\r\n");
			GPIO_SetLow( GPIO_WAKEUP_NXP );
			Timer_Restart( lpcWakeUpTimerId );
			UART_SendDataWithCallback( lpcTxBuf, lpcDataLen, BLEAnchor_UartCallback, NULL );
			status = Q_TRAN(Serial_Xmit_State);
			break;


		  default:
			// Let the super class handle this.

		//	printf("DEFAULT %d\r\n", Q_SIG(me));
			status = Q_SUPER(QHsm_top);
			break;
		}
		return status;
}
/**
 *
 */
QState Serial_Xmit_State(Serial_t *me)
{
	QState status;

	switch( Q_SIG(me) ) {
	   /**
	 	 * BLE_ANCHOR_LORA_READY_SIG signal is sent by BLEAnchor_UartCallback()
	 	 * to indicate LORA module is ready to receive transport messages.
	  	*/
	  case SERIAL_TASK_XMIT_COMPLETED_SIG:
	  case SERIAL_TASK_LPC_WAKEUP_TIMEOUT_SIG:
		//  printf("XMIT_COMPLETED\r\n");
	      // re-enable automatic sleep mode
	      PML_AutomaticSleepModeDisable(PML_AUTO_SLEEP_DISABLE_HCI, false);
	      Timer_Disable( lpcWakeUpTimerId );
//	      Timer_Restart( wdtResetTimer );
	      // start connectable advertisement
	//      QACTIVE_POST(&gBeaconTask.super, BLE_ANCHOR_START_CONN_ADV_SIG, 0u);
	      status = Q_TRAN(Serial_Idle_State);
		  break;

	  case SERIAL_TASK_RESET_WDT_SIG:
		//  printf("Reset watchdog\r\n");
	  //    Watchdog_ResetTimer();
	      status = Q_HANDLED();
	      break;


	   default:
		// Let the super class handle this.
		   status = Q_SUPER(QHsm_top);
		   break;
		}
	return status;
}
/**
 *
 */
/********************************************************************************
 *
 *  Serial_UartCallback
 *
 ********************************************************************************/
void BLEAnchor_UartCallback(Driver_Status_t status, void *pUserData)
{
	QK_ISR_ENTRY();

    ( void ) status;
    ( void ) pUserData;


    /* send signal to indicate LORA module is ready */
    if( DRIVER_STATUS_RX == status){
    	if( uartRxChar == LORA_TRANSPORT_HEADER_BYTE)
    		QACTIVE_POST_ISR(&gSerialTask.super, SERIAL_TASK_LORA_READY_SIG, 0u);
    	UART_ReceiveDataWithCallback( &uartRxChar, UART_RX_COUNT_ON_LORA_RDY, BLEAnchor_UartCallback, NULL );
    }

    else if( DRIVER_STATUS_TX == status){
        	QACTIVE_POST_ISR(&gSerialTask.super, SERIAL_TASK_XMIT_COMPLETED_SIG, 0u);
    }
    QK_ISR_EXIT();
}
