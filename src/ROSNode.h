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
    const uint32_t deadline            = 5000;   // 5 ms default deadline (Can be overridden by derived classes)
    const uint32_t max_deadline_misses = 2;      // The maximum number of deadline misses before the task is dropped
    uint32_t execution_time      = 0;            // The time it takes to execute the task
    uint32_t deadline_misses     = 0;            // Number of times the deadline has been missed
    boolean  dropped_task        = false;

    /**
     * This method is called by the main loop to run the update() method of this node
     * It ensures that this node does not violate the real-time constraints of the system
     * @note This method shall not be overridden
     * @note This does method does not protect against a stuck task. It can only function if the task returns
     * @see update()
     */
    virtual void run_update() final {
        uint32_t start_time = micros();
        this->update();
        this->execution_time = micros() - start_time;
        if (this->execution_time > this->deadline) {
            this->deadline_misses++;
            if (this->deadline_misses > this->max_deadline_misses) {
                this->dropped_task = true;
            }
        }
    }

//    virtual void publish() {
//        // Should be overridden
//    }

    virtual void subscribe(ros::NodeHandle* node_handle) {
        // Should be overridden
    }

    virtual void update() {
        // Should be overridden
    }
};


#endif //TEENSYCANTRANSCEIVER_ROSNODE_H
