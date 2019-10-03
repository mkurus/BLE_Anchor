/**
 *
 */

#define      BLE_TRACKER_IDLE_TIMEOUT             (float)5000.0


#define      BLE_TRACKER_SCAN_PERIOD_IDLE         (float)(60.0 * 5.0)
#define      BLE_TRACKER_SCAN_PERIOD_MOVING       (float)(30.0)


typedef struct
{
    // Root structure for all state machines.
    QActive super;
} Serial_t;




void Serial_Constructor(Serial_t *me);
