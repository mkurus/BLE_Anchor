#ifndef BLE_TRACKER_PLATFORM_H_
#define BLE_TRACKER_PLATFORM_H_

#ifndef __BLEBASE_H
#include <BleBase.h>
#endif

#ifndef __BLEENGINE_H
#include <BleEngine.h>
#endif



/**
 *
 */
typedef struct
{
    // Root structure for all state machines.
    QActive super;

    // Handler for the Bluetooth Low Energy stack.
    BleHandler handler;
} Beacon_t;


#endif   /*BLE_TRACKER_PLATFORM_H_ */
