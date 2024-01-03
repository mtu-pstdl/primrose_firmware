//
// Created by Jay on 7/3/2023.
//

#ifndef PRIMROSE_MCIU_ESTOPDEVICE_H
#define PRIMROSE_MCIU_ESTOPDEVICE_H

/**
 * EStopDevice is an abstract class that provides an interface for all EStop devices to implement
 * Allowing for the EStopController to check the health of all EStop devices and trigger an EStop if necessary
 * @note Not all EStop devices have an estop action and may only be used to monitor the health of the device
 * @note Additionally, not all EStop devices are used to trigger an EStop and may only take action when an EStop is triggered
 */
class EStopDevice {

public:

    // Allow for some granularity in the type of action that an EStop device can take
    enum TRIP_LEVEL {
        NO_FAULT = 0,  // No fault detected
        WARNING = 1,   // Fault detected, but no EStop required
        FAULT = 2,     // Fault detected, EStop required
    };

    /**
     * This function is called by the EStopController when an estop is triggered.
     */
    virtual void estop() {
        // Should be overridden
    }

    /**
     * This function is called by the EStopController to determine if this device has tripped an estop.
     * If this function returns true, the EStopController will then call the estop() function on all other devices.
     */
    virtual TRIP_LEVEL tripped(char* device_name, char* device_message) {
        // Should be overridden
    }

    /**
     * This function is called by the EStopController when the estop is cleared.
     */
    virtual void resume() {
        // Should be overridden
    }

};

#endif //PRIMROSE_MCIU_ESTOPDEVICE_H
