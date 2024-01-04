//
// Created by Jay on 5/26/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ROSNODE_H
#define TEENSYCANTRANSCEIVER_ROSNODE_H


#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"

/**
 * This class allows for iteration over all the ROSNode objects in the project. It is used in the main loop
 * to call the update() method of all the ROSNode objects.
 * @note This class is not meant to be instantiated. It is only meant to be inherited from.
 */
class ROSNode {

public:

    // Values to be used by the main loop to drop tasks that violate the real-time constraints
    uint32_t execution_time      = 0;
    uint32_t deadline            = 5000;   // 5 ms default deadline (Can be overridden by derived classes)
    uint32_t deadline_misses     = 0;      // Number of times the deadline has been missed
    uint32_t max_deadline_misses = 5;      // The maximum number of deadline misses before the task is dropped
    boolean  dropped_task        = false;

    virtual void publish() {
        // Should be overridden
    }

    virtual void subscribe(ros::NodeHandle* node_handle) {
        // Should be overridden
    }

    virtual void update() {
        // Should be overridden
    }
};


#endif //TEENSYCANTRANSCEIVER_ROSNODE_H
