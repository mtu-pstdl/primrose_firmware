//
// Created by Jay on 7/3/2023.
//

#ifndef PRIMROSE_MCIU_ESTOPDEVICE_H
#define PRIMROSE_MCIU_ESTOPDEVICE_H

class EStopDevice {

public:

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
    virtual bool tripped() {
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
