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

    enum ros_commands {
        ESTOP = 0,
        RESUME = 1,
        ENABLE_AUTOMATIC = 2,
        DISABLE_AUTOMATIC = 3,
    };

    ros::Subscriber<std_msgs::Int32, EStopController> estop_sub;

    void estop_callback(const std_msgs::Int32& msg);

    // An array of pointers to estop devices
    EStopDevice* estop_devices[15] = {nullptr};

    // Automatic E-Stop variables
    boolean         automatic_estop_enabled = true;
    boolean         automatic_estop_inhibited = false;
    EStopDevice*    tripped_device = nullptr;  // The device that tripped the E-Stop

    // E-Stop variables
    boolean  estop_triggered = false;
    uint32_t estop_triggered_time = 0;
    uint32_t estop_resume_time = 0;  // The time to wait after the contactor is closed before the E-Stop is cleared


    void check_for_faults() {
        if (!automatic_estop_enabled || automatic_estop_inhibited) {
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
        pinMode(MAIN_CONTACTOR_PIN, OUTPUT);
    }

    void add_estop_device(EStopDevice* estop_device) {
        for (auto & i : estop_devices) {
            if (i == nullptr) {
                i = estop_device;
                return;
            }
        }
    }

    static bool is_high_voltage_enabled() {
        return digitalReadFast(MAIN_CONTACTOR_PIN);
    }

    void trigger_estop() {
        for (auto & estop_device : estop_devices) {
            if (estop_device != nullptr) {
                estop_device->estop();
            }
        }
    }

    void resume() {
        digitalWrite(MAIN_CONTACTOR_PIN, HIGH);
        estop_triggered = false;
        estop_triggered_time = 0;
        estop_resume_time = millis();
        this->automatic_estop_inhibited = true;
    }

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(estop_sub);
    }

    void update() override {
        if (!estop_triggered) {
            if (millis() - estop_resume_time > 3000)
                this->automatic_estop_inhibited = false;
            this->check_for_faults();
        } else {
            if (millis() - estop_triggered_time > ESTOP_CONTACTOR_DELAY)
                digitalWriteFast(MAIN_CONTACTOR_PIN, LOW);  // Open the main contactor
        }
    }

};


#endif //PRIMROSE_MCIU_ESTOPCONTROLLER_H
