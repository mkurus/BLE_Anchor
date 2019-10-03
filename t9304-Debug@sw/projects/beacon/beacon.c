////////////////////////////////////////////////////////////////////////////////
///
/// @file       patches/beacon/beacon.c
///
/// @project    T9304
///
/// @brief      Example Bluetooth Low Energy beacon application.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2016 EM Microelectronic
/// @cond
///
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
/// 1. Redistributions of source code must retain the above copyright notice,
/// this list of conditions and the following disclaimer.
/// 2. Redistributions in binary form must reproduce the above copyright notice,
/// this list of conditions and the following disclaimer in the documentation
/// and/or other materials provided with the distribution.
///
////////////////////////////////////////////////////////////////////////////////
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
/// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
/// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
/// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
/// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
/// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// @endcond
////////////////////////////////////////////////////////////////////////////////

// EM Includes
#include <types.h>
#include <macros.h>
#include <em_qpn.h>
#include <bsp.h>

#include <platform.h>
#include <pml.h>
#include <boot.h>
#include <spi_slave.h>
// Standard Includes
#include <string.h>
#include <timer_hal.h>
#include <t9304_uni_tim.h>
#include <timer.h>
#include <radio.h>
#include <watchdog.h>
// Stack Includes
#include <BleBase.h>
#include <BleGap.h>
#include <BleEngine.h>
#include <string.h>
#include <BleHostStackTask.h> // To access AO_BleHostStackTask
#include <i2c.h>
#include <i2c_hal.h>
#include "beacon.h"
#include "signals.h"
#include "anchor_bsp.h"
#include "i2c_main.h"
#include <uart.h>
#include <printf.h>
#include <string.h>
#include <stdlib.h>
#include "lis2dh12.h"
#include "lis2dh12_platform.h"
#include "serial_task.h"

/**
 *
 */
#define     Device_ID					   0xB1				// Lora packet device ID

#define     ANCHOR_BD_ADDR_0			   'T'
#define     ANCHOR_BD_ADDR_1			   'R'
#define     ANCHOR_BD_ADDR_2			   'I'



#define      TPL5010_DONE_DELAY                      1000.0
#define      TPL5010_REXT_CALCULATE_TIMEOUT          250.0
#define      BLE_DISCONN_TIMEOUT                     500.0
#define      BLE_CONN_TIMEOUT                        1000.0                // BLE connection timeout
#define      LPC_MAX_WAKEUP_TIME                     30.0

#define      BLE_PAYLOAD_SIZE						29
#define      BLEINFOTYPE_SAVED_CLIENTCONFIG       ( 0xFF )


#define     BLEADDRESS_LENGTH_MINI              	3
#define     MAX_ANCHOR_COUNT						4					// maximum anchor count to send
#define     OUTPUT_POWER_LEVEL			            11					// RF TX power level.
#define     BLE_TRACKER_MAX_REMOTE_SERVICES 		5
#define     TIMER_SET_SCAN_TIME			            1  					// Tracker scan time : 5 sec
#define     ANCHOR_INQURTY_TIME                    1000                 // default anchor inquiry duration
#define     INQUIRY_PERIOD_IDLE                     10
#define     INQUIRY_PERIOD_MOVING                    3
#define     MAX_ANCHOR_LIST_COUNT					50					// maximum anchor count to add the list
#define 	MIN_RSSI_VALUE					       -85					// RSSI Filter
#define     LED_ACTIVE_DURATION                     5.0
#define     LED_DEACTIVE_DURATION                   50.0                // duration between multiple led blinks
#define     NUM_CONNECTION_LED_BLINK                2                   // Number of led blinks for successfull connection
/**
 * Type Definitions
 */
typedef struct
{
    bool           isConnectRequested;
    BD_ADDR        addrOfDeviceToConnect;
    BleAddressType addrType;
    BleGapMode     mode;
    U8             numberOfServices;
    U16            services[ BLE_TRACKER_MAX_REMOTE_SERVICES ];
} BLE_TrackerRemoteDeviceContext_t;

// anchor list
typedef struct
{
	BD_ADDR			bd_addr;
	S8				rssi;
	BleAddressType 	addrType;
	bool			isConnectable;
	U8				sampleCount;
	bool 			sorted;
}AnchorList_t;

/**
 *  Anchor payload
 */
typedef struct
{
	U8				bd_addr_Anchor[BLEADDRESS_LENGTH_MINI];
	S8				rssi;
}AnchorPayload_t;

/**
 *
 */
typedef struct
{
	U8					PackageSize;
	U8					DeviceType;
	U8					BatteryLevel;
	U8					Tracker_bd_addr[BLEADDRESS_LENGTH_MINI];
	AnchorPayload_t		AnchorPayload[MAX_ANCHOR_COUNT];
}OfflineTrackerPayload_t;


/**
 * Global variables
 */
AnchorList_t Anchor_List[MAX_ANCHOR_LIST_COUNT];
AnchorList_t Anchor_List_Sorted[MAX_ANCHOR_LIST_COUNT];
AnchorPayload_t Anchor_Payload[MAX_ANCHOR_COUNT];
OfflineTrackerPayload_t Offline_Tracker_Payload;
BLE_TrackerRemoteDeviceContext_t gRemoteDeviceContext;

/**
 * Local BLE address
 */
BD_ADDR       *gpLocalAddr;
BleAddressType gLocalAddrType;

BleGattClient 		gattClient;
BleGattHandler 		gattHandler;
BleGattCommand 		gattCommand;
AttHandle 			valueHandle;


AttServiceAttribute serialService;                     // The main service memory

AttAttribute writeAttribute;                           // The write attribute value
AttAttribute readAttribute;

AttInstantiatedAttribute serialClientConfigAttribute;  //the CCCD

Att128BitCharacteristicAttribute writeCharacteristicValue;
Att128BitCharacteristicAttribute readCharacteristicValue;

U8 serialClientConfigMemory[ 2 ];
U8 writeValue[ BLE_PAYLOAD_SIZE ];          // The Attribute Value
U8 readValue[  BLE_PAYLOAD_SIZE ];

Lis2dh12_register *ptrlis2dh12_register;
Lis2dh12_register lis2dh12_register[2];
Lis2dh12_config lis2dh12_config[2];
Lis2dh12_output lis2dh12_output[2];
DefProjectSettings defProjectSettings;


char gpLocalDeviceName[16];
uint16_t numUartTxChar = 0;             // number of bytes transmitted to LORA module
uint16_t lpcDataLen;                    // number of bytes to be transmitted to LORA module

char lpcTxBuf[BLE_PAYLOAD_SIZE];
U8 manSpecData; //= { 0x03 };
U8 gDisconnectReason;
U16 advInterval;
U16 connHandle;
U8 numConnLedBlinks = 0;
U8 Anchor_Count = 0;
U8 ReConnect_Counter = 0;
bool Connecting_to_An_Anchor = false;
bool Connected_to_an_Anchor = false;
bool Try_to_ReConnect = false;

/* timer IDs*/
int8_t BleConnectTimerId;
int8_t BleDisconnectTimerId;
int8_t inquiryTimer;
int8_t lpcWakeUpTimerId;
int8_t wdtResetTimer;
int8_t tpl5010DoneDlyTimer;

/**
 * Constant variable declarations
 */
// 128 bit Read characteristic UUID (0x6e400003-b5a3-f393-e0a9-e50e24dcca9e)
//CONST_DECL U8 writeUUIDValue[] = { 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e };

// custom service UUID
// 128 bit UUID is 0x6e400001-b5a3-f393-e0a9-e50e24dcca9e
CONST_DECL U8 anchorServiceUUIDValue[] = { 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5,
        0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e };

// Read characteristic UUID
// 128 bit UUID is 0x6e400003-b5a3-f393-e0a9-e50e24dcca9e
CONST_DECL U8 writeUUIDValue[] = { 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5,
        0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e };

// Write characteristic UUID
// 128 bit UUID is 0x6e400002-b5a3-f393-e0a9-e50e24dcca9e
CONST_DECL U8 readUUIDValue[] = { 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5,
        0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e };
/**
 * External declarations
 */
extern UART_Configuration_t gUART_Config;
extern I2C_Configuration_t  gI2C_Config;
extern SPIS_Configuration_t gSPIS_Config;
extern const Config_ModuleConfiguration_t gPML_Config;
extern void tiny_putc(void* p, char c);


extern uint8_t I2C_Buffer;
extern Serial_t gSerialTask;
extern QEvt gSerialEventQueue[5];
/**
 * Function prototypes
 */
void  Anchor_AttServiceCallback( AttServerCallbackParms *serverCallbackParms );
void  ConfigureNXPWakeUpGPIO();
void  ConfigureLedGPIO();
void  ConfigureAccInterruptGPIO();
void  ConfigureUartGPIO();
bool  MotionSensor_Init();
bool  AnchorService_Register();
void  BlockingDelay_us(uint16_t us);
void  ConfigureTPL5010GPIO();

uint8_t checkIC_Lis2dh12(lis2dh12_devList dev);
uint8_t lis2dh12_SendData(uint8_t addrReg, uint8_t *ptrRegister, uint8_t checkCntrl);
uint8_t lis2dh12_ReadData(uint8_t regAddr, uint8_t *rxBuffPtr);


/**
 * Timer callback functions
 */
void cbLpcWakeUpTimeout();
void cbConnTimeout();
void cbDisConnTimeout();
void cbLedActiveTimeout();
void cbLedInactiveTimeout();
void cbWdtResetTimer();
void cbRextCalculateTimeout();
void cbDoneDelayTimer();


/**
 *
 */
_Static_assert(255 >= (int)BEACON_SIG_COUNT, "Too many QP-nano signals!");

// This variable represents the task.
Beacon_t gBeaconTask;

// This variable stores all signals being sent to the task.
static SECTION_PERSISTENT QEvt gBeaconEventQueue[20];

static QMutex Beacon_StackMutexLock(void)
{
    // For some reason this does not work.
    return QK_mutexLock(0xFF /*AO_BleHostStackTask.super.prio*/);
}

static void Beacon_StackMutexUnlock(QMutex mutex)
{
    QK_mutexUnlock(mutex);
}

/**
 * @bried Called during a stack event.
 */
void Beacon_CoreCallback(BleEvent event, BleStatus status, void* pParam);

/**
 * @bried Called during a GAP event.
 */
void Beacon_GapCallback(BleGapEvent event, BleStatus status, void *parms);

/**
 * @brief Constructor for the task.
 *
 * This will initialize the state machine variable @ref gBeaconTask.
 *
 * @param me Pointer to the task to construct.
 */
void Beacon_Constructor(Beacon_t *me);

/**
 * @brief Initialization state.
 *
 * The first state of a QP-nano task is special. It must always make a
 * transition to another state. This implementation will initialize the
 * Bluetooth Low Energy stack.
 *
 * @param me Pointer to the task.
 * @returns Next action for the state machine to take (stay or transition).
 */
QState Beacon_Init(Beacon_t *me);

/**
 * @brief Waiting for Bluetooth Low Energy stack initialization.
 *
 * @param me Pointer to the task.
 * @returns Next action for the state machine to take (stay or transition).
 */
QState Beacon_WaitForStack(Beacon_t *me);

/**
 * @brief Main state of the beacon application.
 *
 * @param me Pointer to the task.
 * @returns Next action for the state machine to take (stay or transition).
 */
QState BLE_Anchor_Disconnecting_State(Beacon_t *me);
QState BLE_Anchor_Disconnected_State(Beacon_t *me);
QState BLE_Anchor_Connected_State(Beacon_t *me);
QState BLE_Anchor_Wait_RExt(Beacon_t *me);

/**
 * @brief Where programs go to die.
 *
 * @param me Pointer to the task.
 * @returns Next action for the state machine to take (stay or transition).
 */
QState Beacon_Error(Beacon_t *me);

////////////////////////////////////////////////////////////////////////////////

void Beacon_CoreCallback(BleEvent event, BleStatus status, void *pParam)
{
    switch(event)
    {
        case BLEEVENT_INITIALIZATION_RSP:
            // Notify the task.
            QACTIVE_POST(&gBeaconTask.super, (int)BEACON_BLE_INIT_SIG,
                (QParam)status);
            break;

        default:
            break;
    }
}


void Beacon_GapCallback(BleGapEvent event, BleStatus status, void *parms)
{
    BleGapConnectionInformation *pConnectionInfo;
    char verStr[128];

    switch(event)
    {
        case BLEGAP_EVENT_CONNECTED:
        	pConnectionInfo = ((BleGapConnectionInformation *) parms );
         	connHandle = ((BleGapConnectionInformation *) parms )->connHandle;
         	BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE | BLEMODE_NOTCONNECTABLE, ADVERTISE_INTERVAL_TIME);

         	printf("\n***********\n");

         	sprintf(verStr, "BLE Anchor Software Rev: V00%d", ANCHOR_VERSION);

#if( USE_EXTERNAL_WDT == 1)
         	strcat(verStr, "w");
#endif

#if (USE_OFFLINE_ANCHOR == 1)
         	strcat(verStr, "_off\n");
#else
         	strcat(verStr, "_on\n");
#endif
            printf(verStr);

         	printf("Connection from...");
         	printf(" %02X", pConnectionInfo->remoteAddress.addr[0]);
         	printf(":%02X", pConnectionInfo->remoteAddress.addr[1]);
         	printf(":%02X", pConnectionInfo->remoteAddress.addr[2]);
         	printf(":%02X", pConnectionInfo->remoteAddress.addr[3]);
         	printf(":%02X", pConnectionInfo->remoteAddress.addr[4]);
         	printf(":%02X\r\n", pConnectionInfo->remoteAddress.addr[5]);

            QACTIVE_POST(&(gBeaconTask.super), BLE_ANCHOR_TRACKER_CONNECTED_SIG,  ( const QParam )parms );
            break;

        case BLEGAP_EVENT_DISCONNECTED:
        case BLEGAP_EVENT_CONNECTIONCANCELED:
        	{
        		BleGapConnectionInformation *connInfo = (BleGapConnectionInformation *) parms;
        		gDisconnectReason = ( U8 ) connInfo->disconnectionReason;
        	}
#if  !USE_OFFLINE_ANCHOR
        	BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE | BLEMODE_CONNECTABLE, ADVERTISE_INTERVAL_TIME);
#else
            BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE, ADVERTISE_INTERVAL_TIME);
#endif
            QACTIVE_POST(&(gBeaconTask.super), BLE_ANCHOR_TRACKER_DISCONNECTED_SIG,  gDisconnectReason  );
            break;


        default:
            break;
    }
}
/**
 *
 */
void Beacon_Constructor(Beacon_t *me)
{
    // Tell QP-nano what the initial state is.
    QActive_ctor(&me->super, Q_STATE_CAST(Beacon_Init));
}


QState Beacon_Init(Beacon_t *me)
{
    QState status;
    QMutex mutex;
    char verStr[128];

    // Protect calls to the stack.
    mutex = Beacon_StackMutexLock();

    if(BLESTATUS_PENDING == BLEMGMT_Init())
    {
        // Register a stack callback.
        me->handler.callback = Beacon_CoreCallback;

        // Check if the callback was registered properly.
        if(BLESTATUS_SUCCESS != BLEMGMT_RegisterHandler(&me->handler))
        {
            // There was an error.
            status = Q_TRAN(Beacon_Error);
        }
        else
        {
            // Now wait for the stack to finish initialization.
            status = Q_TRAN(Beacon_WaitForStack);
        }
    }
    else
    {
        // The call to BLEMGMT_Init() failed.
        status = Q_TRAN(Beacon_Error);
    }

   /* disable JTAG pin functionality */
    GPIO_DisableJtag();

    int i= 0;
    for (i = 0; i < 12; i++){
    /*   if( (i == I2C_SDA_GPIO) ||  (i == I2C_SCK_GPIO) || (i == UART_TX_GPIO) )
        	continue;
       else{*/
        	GPIO_SetOutputPinFunction(i, GPIO_PIN_FUNC_OUT_GPIO);
        	GPIO_DisableOutput(i);
        	GPIO_DisablePullUp(i);
        	GPIO_EnablePullDown(i);
    //   }
    }

    ConfigureLedGPIO();
    ConfigureNXPWakeUpGPIO();

#if ( USE_UART == 1 )
      ConfigureUartGPIO();
#endif

#if  (USE_EXTERNAL_WDT == 1)
      ConfigureTPL5010GPIO();
      GPIO_SetLow(TPL5010_DONE_GPIO);
      printf("TPL5010 Pin configuration Done\r\n");
#endif

#if ( USE_ACCELEROMETER_I2C == 1 )
      ConfigureAccInterruptGPIO();

      if( MotionSensor_Init()){
    	  set_AccINT1EnbDsb( INT_X|INT_Y|INT_Z, DEV0_LIS, UNCHECK_WRITTEN_DATA);
    	  printf("Motion Sensor Detected\r\n");
      }
      else
           printf("Cannot detect motion sensor!\n");
#endif

  	  sprintf(verStr, "BLE Anchor Software Rev: V00%d", ANCHOR_VERSION);

  #if( USE_EXTERNAL_WDT == 1)
         strcat(verStr, "w");
  #endif

  #if (USE_OFFLINE_ANCHOR == 1)
         strcat(verStr, "_off\n");
  #else
         strcat(verStr, "_on\n");
  #endif
         printf(verStr);

    printf("Initialization Completed\n");

    PML->RegPmlPadWake.r16[0] = 0;
    gRemoteDeviceContext.isConnectRequested = false;

	/* initialize a GATT client*/
/*	gattHandler.callback = GATT_Callback;
	gattClient = BLEGATT_RegisterClient(&gattHandler);*/

    Beacon_StackMutexUnlock(mutex);

    return status;
}
/**
 *
 */
bool MotionSensor_Init()
{
	/* set default accelerometer settings */
	deviceSettings(&defProjectSettings);

	if(checkIC_Lis2dh12(DEV0_LIS)){

		printf("lis2dh12 device OK\r\n");

       /* this function must be called at least once before starting operations with lis2.
	    * the return value must be an address from the Lis2dh12_register structure type in order to avoid any loss of value. */
	    ptrlis2dh12_register = lis2dh12_initDefConfig(DEV0_LIS, CHECK_WRITTEN_DATA);

		// clear the interrupt flag on lis2dh12
	    lis2dh12_readReg_INT1SRC(ptrlis2dh12_register);

	    return true;
	  // mcu wake-up and set lis2 interrupts for mcu int request
	  // !!! we enable the interrupt output after motionless timer times up
	  //set_AccINT1EnbDsb( INT_X|INT_Y|INT_Z, DEV0_LIS, UNCHECK_WRITTEN_DATA);
	}
	else
		return false;
}
/**
 *
 */
void ConfigureNXPWakeUpGPIO()
{
	 GPIO_DisablePullUp(GPIO_WAKEUP_NXP);
	 GPIO_EnablePullDown(GPIO_WAKEUP_NXP);

	 GPIO_EnableOutput(GPIO_WAKEUP_NXP);
	 GPIO_DisableInput(GPIO_WAKEUP_NXP);

	 GPIO_SetOutputPinFunction(GPIO_WAKEUP_NXP, GPIO_PIN_FUNC_OUT_GPIO );
	 GPIO_SetLow(GPIO_WAKEUP_NXP);
}
/**
 *
 */
void ConfigureLedGPIO()
{
	GPIO_DisableInput(GPIO_LED);
	GPIO_EnableOutput(GPIO_LED);
    GPIO_DisablePullDown(GPIO_LED);
    GPIO_DisablePullUp(GPIO_LED);
	GPIO_SetOutputPinFunction(GPIO_LED,  GPIO_PIN_FUNC_OUT_GPIO);
}
/**
 * Configure accelerometer interrupt pin
 */
void ConfigureAccInterruptGPIO()
{
	 GPIO_DisableOutput(GPIO_ACCEL_INT1);
	 GPIO_DisablePullDown(GPIO_ACCEL_INT1);
	 GPIO_EnableInput(GPIO_ACCEL_INT1);
	 GPIO_EnablePullUp(GPIO_ACCEL_INT1);

	 // Setup INT based on SW GPIO pulled low
	 GPIO_SetPolarityFalling(GPIO_ACCEL_INT1);

	 IRQ_Enable(IRQ_GROUP_GPIO, GPIO_MASK(GPIO_ACCEL_INT1));
	 IRQ_Unmask(IRQ_GROUP_GPIO, GPIO_MASK(GPIO_ACCEL_INT1));
}
/**
 *
 */
void ConfigureUartGPIO()
{
	/* configure UART TX pin */
	GPIO_DisableInput(UART_TX_GPIO);
	GPIO_EnablePullUp(UART_TX_GPIO);
	GPIO_DisablePullDown(UART_TX_GPIO);
	GPIO_SetOutputPinFunction(UART_TX_GPIO, GPIO_PIN_FUNC_OUT_UART_TXD);
	GPIO_EnableOutput(UART_TX_GPIO);

	/* configure UART RX pin */
	GPIO_DisableOutput(UART_RX_GPIO);
	GPIO_EnablePullUp(UART_RX_GPIO);
    GPIO_DisablePullDown(UART_RX_GPIO);
    GPIO_SetInputFunctionPin( GPIO_PIN_FUNC_IN_UART_RXD, UART_RX_GPIO);
	GPIO_EnableInput(UART_RX_GPIO);
}

/**
 *
 */
#if  (USE_EXTERNAL_WDT == 1)
void ConfigureTPL5010GPIO()
{
	/* configure done signal pin */
	GPIO_DisableInput(TPL5010_DONE_GPIO);
    GPIO_EnableOutput(TPL5010_DONE_GPIO);
	GPIO_DisablePullDown(TPL5010_DONE_GPIO);
	GPIO_DisablePullUp(TPL5010_DONE_GPIO);

    /* configure wake signal pin as input*/
    GPIO_DisableOutput(TPL5010_WAKE_GPIO);
    GPIO_EnableInput(TPL5010_WAKE_GPIO);
    GPIO_DisablePullDown(TPL5010_WAKE_GPIO);
    GPIO_DisablePullUp(TPL5010_WAKE_GPIO);

    GPIO_SetPolarityRising(TPL5010_WAKE_GPIO);

    IRQ_Enable(IRQ_GROUP_GPIO, GPIO_MASK(TPL5010_WAKE_GPIO));
    IRQ_Unmask(IRQ_GROUP_GPIO, GPIO_MASK(TPL5010_WAKE_GPIO));
}
#endif
/**
 *
 */
uint8_t checkIC_Lis2dh12(lis2dh12_devList device)
{
    if(I2C_ReadData_Blocking(0x0F, (uint8_t*)&I2C_Buffer))
    {
		if(I2C_Buffer == 0x33)
			return 1;
		else
			return 0;
    }
    else
    	return 0;
}
/**
 *   @param checkCntrl: read and compare the written data.
 *   @return Returns non-zero value on successful all controls
 */
uint8_t lis2dh12_SendData(uint8_t addrReg, uint8_t *ptrRegister, uint8_t checkCntrl)
{
	if(I2C_WriteData_Blocking(addrReg, ptrRegister))
		return 1;


	return 0;
}
/*
 * @param regAddr: register address to be reached
 * @param rxBuffPtr: address where the read value will be placed
 * @return Returns non-zero value on successful completion of transfer.
 */

uint8_t lis2dh12_ReadData(uint8_t regAddr, uint8_t *rxBuffPtr)
{
	if(I2C_ReadData_Blocking(regAddr, rxBuffPtr))
		return 1;

	return 0;
}

/**
 *
 */
QState Beacon_WaitForStack(Beacon_t *me)
{
    QState status;
  //  QMutex mutex;


    switch( Q_SIG(me) )
    {
        case BEACON_BLE_INIT_SIG:
        {
            if(BLESTATUS_SUCCESS == Q_PAR(me))
            {

           //     BLEGAP_RegisterDevice( BLEGAPROLE_PERIPHERAL , Beacon_GapCallback );
                BLEGAP_RegisterDevice( BLEGAPROLE_PERIPHERAL | BLEGAPROLE_BROADCASTER, Beacon_GapCallback );

                // Set the name of the peripheral.
                BLEGAP_GetLocalBdAddr( &gpLocalAddr, &gLocalAddrType );

                sprintf( (char*)gpLocalDeviceName, "TrioAnch-%02X%02X%02X", gpLocalAddr->addr[3], gpLocalAddr->addr[4], gpLocalAddr->addr[5] );
                BLEGAP_SetLocalBluetoothDeviceName((U8*)gpLocalDeviceName, strlen(gpLocalDeviceName));

                printf("Local BLE address: ");
                  		    printf(" %02X", gpLocalAddr->addr[0]);
                  		  	printf(":%02X", gpLocalAddr->addr[1]);
                  		  	printf(":%02X", gpLocalAddr->addr[2]);
                  		  	printf(":%02X", gpLocalAddr->addr[3]);
                  		  	printf(":%02X", gpLocalAddr->addr[4]);
                  		  	printf(":%02X\r\n", gpLocalAddr->addr[5]);


                manSpecData = ANCHOR_VERSION;
                /* set manufacturer specific data as version information*/
                BLEGAP_SetManufacturerSpecificData( &manSpecData, sizeof(manSpecData) );

               // Set RF output power
                RF_SetOutputPower(RF_OUTPUT_POWER_NEW, OUTPUT_POWER_LEVEL); 			// EM9304.pdf - Table-19

                // Register the  service.
                AnchorService_Register();

#if !USE_OFFLINE_ANCHOR
			    BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE | BLEMODE_CONNECTABLE, ADVERTISE_INTERVAL_TIME);
#else
				BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE | BLEMODE_NOTCONNECTABLE, ADVERTISE_INTERVAL_TIME);
#endif

                /* create timers */
                BleConnectTimerId = Timer_ExpressoDelay( (float)BLE_CONN_TIMEOUT, cbConnTimeout, NULL );
                BleDisconnectTimerId = Timer_ExpressoDelay( (float)BLE_DISCONN_TIMEOUT, cbDisConnTimeout, NULL );
                lpcWakeUpTimerId = Timer_ExpressoDelay( (float)LPC_MAX_WAKEUP_TIME, cbLpcWakeUpTimeout, NULL );

#if ( USE_EXTERNAL_WDT == 1 )
                tpl5010DoneDlyTimer = Timer_ExpressoDelay((float)TPL5010_DONE_DELAY, cbDoneDelayTimer, NULL);
                Timer_OneshotDelay( (float)TPL5010_REXT_CALCULATE_TIMEOUT, cbRextCalculateTimeout, 0);
                status = Q_TRAN(BLE_Anchor_Wait_RExt);

#else
          //      QACTIVE_POST( &gBeaconTask.super,  BLE_TRACKER_BLE_START_INQUIRY_SIG, 0);
                status = Q_TRAN(BLE_Anchor_Disconnected_State);
#endif
            }
            else
            {
                status = Q_TRAN(Beacon_Error);
            }

            break;
        }
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
QState BLE_Anchor_Wait_RExt(Beacon_t *me)
{
	 QState status;

	    switch( Q_SIG(me) )
	    {

	    	case BLE_ANCHOR_REXT_CALCULATED_SIG:
	    		 printf("R External Calculated\r\n", Q_PAR(me));
	    		 QACTIVE_POST(&(gBeaconTask.super),  BLE_ANCHOR_TPL5010_DONE_SIG, 0);
	    		 status = Q_TRAN(BLE_Anchor_Disconnected_State);
	    	break;


	        default:
	            // Let the super class handle this.
	            status = Q_SUPER(QHsm_top);
	        break;
	    }

	    return status;
}
/**
 * Disconnected state.
 */
QState BLE_Anchor_Disconnected_State(Beacon_t *me)
{
    QState status;
    QMutex mutex;
    char verStr[128];

    switch( Q_SIG(me) )
    {
    	case BLE_ANCHOR_TRACKER_CONNECTED_SIG:
    		mutex = Beacon_StackMutexLock();
    		//printf("Connected\r\n");
  		  	/* set timer to prevent connection stalling */
  		    Timer_Restart( BleConnectTimerId );
  		    status = Q_TRAN( BLE_Anchor_Connected_State );
  		    Beacon_StackMutexUnlock(mutex);
  		    break;

    	case BLE_ANCHOR_START_CONN_ADV_SIG:
    		printf("BLE_ANCHOR_START_CONN_ADV_SIG\r\n");
/*#if  !USE_OFFLINE_ANCHOR
        	BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE | BLEMODE_CONNECTABLE, ADVERTISE_INTERVAL_TIME);
#else
        	BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE, ADVERTISE_INTERVAL_TIME);
#endif*/
        	status = Q_HANDLED();
    		break;


  	   case BLE_ANCHOR_RESET_WDT_SIG:
  		 sprintf(verStr, "BLE Anchor Software Rev: V00%d", ANCHOR_VERSION);

#if( USE_EXTERNAL_WDT == 1)
  	     strcat(verStr, "w");
#endif

#if (USE_OFFLINE_ANCHOR == 1)
  	   strcat(verStr, "_off\n");
#else
  	   strcat(verStr, "_on\n");
#endif
  	   	  printf(verStr);

 #if  !USE_OFFLINE_ANCHOR
  	      BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE | BLEMODE_CONNECTABLE, ADVERTISE_INTERVAL_TIME);
#else
          BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE, ADVERTISE_INTERVAL_TIME);
#endif
  	      status = Q_HANDLED();
  	      break;

#if ( USE_EXTERNAL_WDT == 1 )
	   case BLE_ANCHOR_TPL5010_DONE_SIG:
		  printf("Reset TPL5010\n");
		  GPIO_SetHigh(TPL5010_DONE_GPIO);
		  BlockingDelay_us(100);
		  GPIO_SetLow(TPL5010_DONE_GPIO);
		  status = Q_HANDLED();
		  break;
#endif

        default:
            // Let the super class handle this.
            status = Q_SUPER( QHsm_top );
            break;
    }

    return status;
}
/**
 *
 */
QState BLE_Anchor_Connected_State(Beacon_t *me)
{
	QState status;
	QMutex mutex;

		switch( Q_SIG(me) ) {

			case BLE_ANCHOR_TRACKER_DISCONNECTED_SIG:
				  mutex = Beacon_StackMutexLock();
				//  printf("Disconnected\r\n");
				//  printf("***********\r\n");
				  Timer_Disable( BleConnectTimerId );
				  Timer_Disable( BleDisconnectTimerId );
			//	  QACTIVE_POST(&gBeaconTask.super, BLE_ANCHOR_START_CONN_ADV_SIG, 0u);
				  status = Q_TRAN( BLE_Anchor_Disconnected_State );
				  Beacon_StackMutexUnlock(mutex);
				  break;

			   /**
			    * BLE_ANCHOR_TRACKER_DATA_RECEIVED_SIG signal is sent by attribute service callback
			    * to indicate a new BLE packet has been received.
			    */
			case BLE_ANCHOR_TRACKER_DATA_RECEIVED_SIG:
			//	  printf("BLE_ANCHOR_TRACKER_DATA_RECEIVED_SIG\r\n");
				  BLEGAP_Disconnect( connHandle );
				  Timer_Restart( BleDisconnectTimerId );
				  QACTIVE_POST(&(gSerialTask.super), SERIAL_TASK_WAKEUP_LORA_SIG, (const QParam) 0);
				  status = Q_HANDLED();
				  break;

			   /**
			   * BLE_TRACKER_BLE_CONN_TIMEOUT_SIG signal is sent by cbConnTimeout timer callback to indicate
			   * maximum connection duration has expired.
			   */
			case BLE_ANCHOR_CONN_TIMEOUT_SIG:
				 printf("Connection Timeout\r\n");
				 BLEGAP_Disconnect( connHandle );
				 Timer_Restart( BleDisconnectTimerId );
				 status = Q_HANDLED();
			 	 break;

			case BLE_ANCHOR_DISCONNECT_TIMEOUT_SIG:
				printf("Disconnect Timeout\r\n");
			//	QACTIVE_POST(&gBeaconTask.super, BLE_ANCHOR_START_CONN_ADV_SIG, 0u);
				status = Q_TRAN( BLE_Anchor_Disconnected_State );
			    break;

			case  BLE_ANCHOR_START_CONN_ADV_SIG:
			//	printf("BLE_ANCHOR_START_CONN_ADV_SIG  while Connected\r\n");
/*#if  !USE_OFFLINE_ANCHOR
				BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE | BLEMODE_CONNECTABLE, ADVERTISE_INTERVAL_TIME);
#else
				BLEGAP_SetModeWithIntervals(BLEMODE_DISCOVERABLE, ADVERTISE_INTERVAL_TIME);
#endif*/
				status = Q_HANDLED();
			    break;

#if  ( USE_EXTERNAL_WDT == 1 )
			  case BLE_ANCHOR_TPL5010_DONE_SIG:
				   printf("Reset TPL5010\n");
				   GPIO_SetHigh(TPL5010_DONE_GPIO);
				   BlockingDelay_us(100);
				   GPIO_SetLow(TPL5010_DONE_GPIO);
				   status = Q_HANDLED();
				   break;
#endif


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
void cbLpcWakeUpTimeout()
{
	QK_ISR_ENTRY();
	QACTIVE_POST_ISR(&(gSerialTask.super), ( int )SERIAL_TASK_LPC_WAKEUP_TIMEOUT_SIG, (const QParam) 0);
	QK_ISR_EXIT();
}

/**
 *
 */
void cbWdtResetTimer()
{
	QK_ISR_ENTRY();
	QACTIVE_POST_ISR(&(gBeaconTask.super), BLE_ANCHOR_RESET_WDT_SIG, (const QParam) 0);
	QACTIVE_POST_ISR(&(gSerialTask.super), SERIAL_TASK_RESET_WDT_SIG,(const QParam) 0);
	QK_ISR_EXIT();
}
/**
 *
 */
void cbConnTimeout()
{
	QK_ISR_ENTRY();
	QACTIVE_POST_ISR(&(gBeaconTask.super), ( int )BLE_ANCHOR_CONN_TIMEOUT_SIG, (const QParam) 0);
	QK_ISR_EXIT();
}
/**
 *
 */
void cbDisConnTimeout()
{
	QK_ISR_ENTRY();
	QACTIVE_POST_ISR(&(gBeaconTask.super), ( int )BLE_ANCHOR_DISCONNECT_TIMEOUT_SIG, (const QParam) 0);
	QK_ISR_EXIT();
}

/**
 *
 */
void cbRextCalculateTimeout()
{
	QK_ISR_ENTRY();
	QACTIVE_POST_ISR(&(gBeaconTask.super), BLE_ANCHOR_REXT_CALCULATED_SIG, (const QParam) 0);
	QK_ISR_EXIT();
}
/**
 *
 */
void cbDoneDelayTimer()
{
	QK_ISR_ENTRY();
	QACTIVE_POST_ISR(&(gBeaconTask.super), BLE_ANCHOR_TPL5010_DONE_SIG, 2);
	printf("\r\nGenerate Done signal\r\n");
	QK_ISR_EXIT();
}
/**
 * IRQ Handler for GPIO
 */
#if (USE_EXTERNAL_WDT == 1)
void IRQUserHandler_GPIO(uint8_t gpio)
{

	QK_ISR_ENTRY();

	switch(gpio){

		case TPL5010_WAKE_GPIO:
			Timer_Restart(tpl5010DoneDlyTimer);
			break;
	}

    QK_ISR_EXIT();
}

JLI_OVERRIDE(IRQUserHandler_GPIO);

#endif
/**
 *
 */
void BlockingDelay_us(uint16_t us)
{
	uint16_t delay = us * 5;				// delay value must be bigger than 5 times of the us value

	for(int i=0;i<delay;i++)
		asm("NOP");
}
/**
 *
 */
bool AnchorService_Register()
{
	    AttUuid typeWrite;
	    AttUuid typeRead;

	    //Secure the database
	    ATT_SERVER_SecureDatabaseAccess();

	    // Register The custom service and
	    // add the characteristic.
	    if (BLESTATUS_FAILED == ATT_SERVER_RegisterServiceAttribute(
	            ATTPDU_SIZEOF_128_BIT_UUID,        // UUID length
	            (U8 *) anchorServiceUUIDValue,     // Service UUID Value
				Anchor_AttServiceCallback,  // Service callback
	            &serialService ))
	    {
	        // The service is already registered.
	        ATT_SERVER_ReleaseDatabaseAccess();
	        return BLESTATUS_FAILED;
	    }

	    // Add The custom characteristic, which are typically a proprietary 128 bit UUID
	    typeWrite.size = ATT_UUID_SIZE_128;
	    typeWrite.value.uuid128 = (U8 *) &writeUUIDValue;

	    // Add The custom2 characteristic
	    typeRead.size = ATT_UUID_SIZE_128;
	    typeRead.value.uuid128 = (U8 *) &readUUIDValue;

	    for (int i = 0; i < BLE_PAYLOAD_SIZE; i++) {
	        writeValue[ i ] = 0x00u;
	        readValue[ i ]  = 0x00u;
	    }

	    //Add characteristic custom Read/Write
	    if( BLESTATUS_FAILED == ATT_SERVER_AddCharacteristic(
	            (ATTPROPERTY_READ | ATTPROPERTY_WRITE),
	            (Att16BitCharacteristicAttribute*) &writeCharacteristicValue,
	            &typeWrite,
	            ATT_PERMISSIONS_ALLACCESS,
				BLE_PAYLOAD_SIZE,
	            ( U8 * ) &writeValue,
	            0u,
	            0u,
	            &serialService,
	            &writeAttribute ))
	    {
	        return BLESTATUS_FAILED;
	    }

	    // Add characteristic custom Read/Write/Notify/Indicate
	    if ( BLESTATUS_FAILED == ATT_SERVER_AddCharacteristic(
	            ( ATTPROPERTY_READ | ATTPROPERTY_NOTIFY | ATTPROPERTY_INDICATE ),
	            (Att16BitCharacteristicAttribute*) &readCharacteristicValue,
	            &typeRead,
	            ATT_PERMISSIONS_ALLACCESS,
				BLE_PAYLOAD_SIZE,
	            ( U8 * ) &readValue,
	            0,
	            0,
	            &serialService,
	            &readAttribute ))
	    {
	        return BLESTATUS_FAILED;
	    }

	    // Add characteristic client configuration
	    if( ATTSTATUS_FAILED == ATT_SERVER_AddCharacteristicClientConfig(
	            ( Att16BitCharacteristicAttribute * ) &readCharacteristicValue,
	            (const U8 *) &AttServerClientConfigDefaultValue[0],
	            (U8 *) &serialClientConfigMemory[0],
	            0, // isValuePersistent
	            BLEINFOTYPE_SAVED_CLIENTCONFIG,
	            &serialClientConfigAttribute ))
	    {
	        return BLESTATUS_FAILED;
	    }

	    //Release the database
	    ATT_SERVER_ReleaseDatabaseAccess();

	    return BLESTATUS_SUCCESS;

}
/**
 *
 */
void Anchor_AttServiceCallback( AttServerCallbackParms *serverCallbackParms )
{
    U8 *pValue;
    U16 length;

    switch (serverCallbackParms->event)
    {
        //Write request event
        case ATTEVT_SERVER_WRITE_REQ:
        //	printf("ATTEVT_SERVER_WRITE_REQ \r\n");
            break;

        // Write was completed.
        case ATTEVT_SERVER_WRITE_COMPLETE:
        //	printf("ATTEVT_SERVER_WRITE_COMPLETE \r\n");

            // If the attribute that get the "write complete event" is customAttribute
            if (serverCallbackParms->parms.writeComplete.attribute  == (const AttAttribute*) &writeAttribute )
            {
                // Post an event to send the TX data.
             //   QACTIVE_POST(&gMyTask.super, CUSTOMSERIAL_TX_SIG, 0u);

                ATT_SERVER_SecureDatabaseAccess();

                // Copy the value from the attribute to the TX buffer.
                if ( ATTSTATUS_SUCCESS == ATT_SERVER_ReadAttributeValue(
                        ( const AttAttribute* ) &writeAttribute,
                        &pValue,
                        &length ))  {

                    QACTIVE_POST(&(gBeaconTask.super), BLE_ANCHOR_TRACKER_DATA_RECEIVED_SIG, 0u);
                    /**
                     * Bluetooth göndericisinin belirlediði paket boyutu bilgisini bize dönüyor.
                	 * Ancak paketin gerçek boyutu gelen verinin ilk datasýnda gizli
                     */
                 //   printf("%d bytes BLE data received\r\n", length);

                    memcpy( lpcTxBuf, pValue, length );
                    lpcDataLen = lpcTxBuf[ 0 ] ;

                    for(int i=0; i < lpcDataLen; i++)
                         printf("%02X ", lpcTxBuf[i]);
                    printf("\r\n");
                }
                else
                {
                //    gTxBuffer.count = 0;
                }
                ATT_SERVER_ReleaseDatabaseAccess();

                // disconnect and wait for another tracker...
            //    BLEGAP_Disconnect( gCurrentConnHandle );
            }
            break;

        // Read request event.
        case ATTEVT_SERVER_READ_REQ:
		//	printf("ATTEVT_SERVER_READ_REQ \r\n");
            break;

        // Handle value indication event sent.
        case ATTEVT_SERVER_HVI_SENT:
		//	printf("ATTEVT_SERVER_HVI_SENT \r\n");
            break;

        //Handle value confirmation event.
        case ATTEVT_SERVER_HANDLE_VALUE_CONFIRMATION:
        //	printf("ATTEVT_SERVER_HANDLE_VALUE_CONFIRMATION \r\n");
            break;
    }
}


/**
 *
 */
QState Beacon_Error(Beacon_t *me)
{
    QState status;

    switch( Q_SIG(me) )
    {
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
void BLEAnchor_UartTxCallback(Driver_Status_t status, void *pUserData)
{
	QK_ISR_ENTRY();

    ( void ) status;
    ( void ) pUserData;




    QK_ISR_EXIT();
}
/**
 *
 */
void Beacon_EntryFunction(void)
{
	 // Use the I2C driver.
#if USE_ACCELEROMETER_I2C

	    I2C_ModuleEntry();

	    gI2C_Config.enabled = true;
	    gI2C_Config.gpioSda = I2C_SDA_GPIO; //1u;
	    gI2C_Config.gpioSck = I2C_SCK_GPIO; //0u;
	    gI2C_Config.clockFrequency = (uint8_t)ClockFrequency100;

	    // Register a wake-up action.
	    PML_ConfigWakeUpByGpio(GPIO_ACCEL_INT1, false, true);
	    PML_RegisterWakeUpAction(PML_WAKEUP_ACTION_RUN_HF_XTAL,
	    SET_BOOT_ACTION_FLAGS_GPIO(GPIO_MASK(GPIO_ACCEL_INT1)), true);

#endif

    // Add the task to QP-nano. This must always happen here.
    (void)QF_addTask(&gBeaconTask.super, &gBeaconEventQueue[0],
        Q_DIM(gBeaconEventQueue));

    // Add the task to QP-nano. This must always happen here.
     (void)QF_addTask(&gSerialTask.super, &gSerialEventQueue[0],
         Q_DIM(gSerialEventQueue));

#if USE_ACCELEROMETER_I2C
    (void)QF_addTask(&gMotionTask.super, &gMotionEventQueue[0],
        Q_DIM(gMotionEventQueue));
#endif


	PML_Configuration_t *pPML_Config = (PML_Configuration_t*) MODIFY_CONST (&gPML_Config);
    pPML_Config->sleepModeForbiden =false;

    Timer_Configuration_t *pTimer_Config = (Timer_Configuration_t*)MODIFY_CONST(&gTimer_Config);
    pTimer_Config->enabled =true;

	gSPIS_Config.spisEnabled = false;

	// Initialize the transport with the SPI slave disabled. The compiler
	// will issue a warning about the buffers being deprecated. This can
	// be ignored because the buffers are not being used in this instant.
	 SPIS_InitBuffers(gSPIS_Config.pRxBuffer, gSPIS_Config.rxBufferSize, gSPIS_Config.pTxBuffer, gSPIS_Config.txBufferSize);

	 Watchdog_SetLimitAndEnable(1000u);  // 10 seconds in 10 ms increments
    // Setup the task during a reset.
    if(!PML_DidBootFromSleep()){

        // Construct our new QP-nano task.
        Beacon_Constructor(&gBeaconTask);
        Serial_Constructor(&gSerialTask);


#if (USE_ACCELEROMETER_I2C == 1)
        Motion_Constructor(&gMotionTask);
#endif

#if (USE_UART == 1)

	    init_printf(NULL, tiny_putc);
#endif


    }

#if (USE_UART == 1)
      // Configure the UART without flow control.
     gUART_Config.configBits.bits.flowCtrlEn = 0u;
     gUART_Config.uartEnabled = true;

#endif

#if (USE_EXTERNAL_WDT == 1)
	    // Register a wake-up action.
	  	PML_ConfigWakeUpByGpio(TPL5010_WAKE_GPIO, true, true);
	  	PML_RegisterWakeUpAction(PML_WAKEUP_ACTION_RUN_HF_XTAL,
	  	SET_BOOT_ACTION_FLAGS_GPIO(GPIO_MASK(TPL5010_WAKE_GPIO)), true);
#endif


    // Override some configuration settings so a configuration patch
    // does not need to be loaded.
    Platform_Configuration_t *pPlatform_Config = (Platform_Configuration_t*)MODIFY_CONST(&gPlatform_Config);

    pPlatform_Config->bleAddr.octets[ 0 ] = ANCHOR_BD_ADDR_0;
    pPlatform_Config->bleAddr.octets[ 1 ] = ANCHOR_BD_ADDR_1;
    pPlatform_Config->bleAddr.octets[ 2 ] = ANCHOR_BD_ADDR_2;

    pPlatform_Config->bleNumberOfConnectionsMax = 2;
    pPlatform_Config->blePacketPayloadLengthMax = 251;
    pPlatform_Config->bleIsStackRequested = 1;
}

ENTRY_FUNCTION(Beacon_EntryFunction);
