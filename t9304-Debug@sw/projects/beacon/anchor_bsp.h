
#define     ANCHOR_VERSION                    14
//#define     USE_OFFLINE_ANCHOR                 1

/**
 * BLE payload size
 */
#define     BLE_PAYLOAD_SIZE			    29
/**
 * Optional features
 */
#define     USE_ACCELEROMETER_I2C           0
#define     USE_UART                        1
#define     USE_LED                         0
#define     USE_EXTERNAL_WDT                0
/**
 * GPIO pin definitions
 */
#define      GPIO_LED                      4
#define      I2C_SDA_GPIO                  1
#define      I2C_SCK_GPIO                  0
#define      UART_RX_GPIO                  6
#define      UART_TX_GPIO                  7

#define      GPIO_WAKEUP_NXP 		       3
#define      GPIO_ACCEL_INT1 			   2

#if   ( USE_EXTERNAL_WDT == 1 )
#define      TPL5010_WAKE_GPIO             9
#define      TPL5010_DONE_GPIO             10
#endif

#define     ADVERTISE_INTERVAL_TIME		  160	                // 100ms					// 32 = 20ms / 16384 = 10.24sn
