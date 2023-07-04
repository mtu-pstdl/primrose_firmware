//
// Created by Jay on 7/3/2023.
//

#ifndef PRIMROSE_MCIU_ESTOPCONTROLLER_H
#define PRIMROSE_MCIU_ESTOPCONTROLLER_H

#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32.h"
#include "EStopDevice.h"

#define MAIN_CONTACTOR_PIN 0

// The time in milliseconds to wait after an estop is triggered before the main contactor is opened (to reduce back EMF)
#define ESTOP_CONTACTOR_DELAY 2500  // 2.5 seconds

class EStopController : public ROSNode {

private:

    ros::Subscriber<std_msgs::Int32, EStopController> estop_sub;

    void estop_callback(const std_msgs::Int32& msg);

    EStopDevice* estop_devices[10] = {nullptr};

    boolean  automatic_estop_enabled = true;

    boolean  estop_triggered = false;
    uint32_t estop_triggered_time = 0;


    void check_for_faults() {
        if (!automatic_estop_enabled) {
            return;
        }
        for (auto & estop_device : estop_devices) {
            if (estop_device != nullptr) {
                if (estop_device->tripped()) {
                    this->trigger_estop();
                    return;
                }
            }
        }
    }

public:

    EStopController() :
        estop_sub("/mciu/estop_controller", &EStopController::estop_callback, this) {
    }

    void add_estop_device(EStopDevice* estop_device) {
        for (auto & i : estop_devices) {
            if (i == nullptr) {
                i = estop_device;
                return;
            }
        }
    }

    void trigger_estop() {
        for (auto & estop_device : estop_devices) {
            if (estop_device != nullptr) {
                estop_device->estop();
            }
        }
    }

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(estop_sub);
    }

    void update() override {
        if (!estop_triggered) {
            this->check_for_faults();
        } else {
            if (millis() - estop_triggered_time > ESTOP_CONTACTOR_DELAY) {
                digitalWrite(MAIN_CONTACTOR_PIN, LOW);
            }
        }
    }

};


#endif //PRIMROSE_MCIU_ESTOPCONTROLLER_H
