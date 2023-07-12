//
// Created by Jay on 7/3/2023.
//

#ifndef PRIMROSE_MCIU_ESTOPCONTROLLER_H
#define PRIMROSE_MCIU_ESTOPCONTROLLER_H

#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32.h"
#include "EStopDevice.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/KeyValue.h"

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
    diagnostic_msgs::DiagnosticStatus* diagnostic_topic;

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

    char* message_string = new char[20];

public:

    EStopController(diagnostic_msgs::DiagnosticStatus* status) :
        estop_sub("/mciu/estop_controller", &EStopController::estop_callback, this) {
        this->diagnostic_topic = status;

        // Setup the diagnostic topic
        this->diagnostic_topic->name = "E-Stop";
        this->diagnostic_topic->hardware_id = "MCIU";
        this->diagnostic_topic->message = message_string;
        sprintf(this->diagnostic_topic->message, "Initializing");

        this->diagnostic_topic->values_length = 4;
        this->diagnostic_topic->values = new diagnostic_msgs::KeyValue[4];
        this->diagnostic_topic->values[0].key = "E-Stop Active";
        this->diagnostic_topic->values[1].key = "Auto E-Stop Enabled";
        this->diagnostic_topic->values[2].key = "Triggering Device";
        this->diagnostic_topic->values[3].key = "Reason";

        // Assign the strings
        for (int i = 0; i < this->diagnostic_topic->values_length; i++) {
            this->diagnostic_topic->values[i].value = new char[20];
            sprintf(this->diagnostic_topic->values[i].value, "Initializing");
        }

        pinMode(MAIN_CONTACTOR_PIN, OUTPUT);
        digitalWrite(MAIN_CONTACTOR_PIN, HIGH);
        this->trigger_estop();
    }

    void add_estop_device(EStopDevice* estop_device) {
        for (auto & i : estop_devices) {
            if (i == nullptr) {
                i = estop_device;
                return;
            }
        }
    }

    bool is_high_voltage_enabled() {
        return !this->estop_triggered;
//        return digitalReadFast(MAIN_CONTACTOR_PIN);
    }

    void trigger_estop() {
        for (auto & estop_device : estop_devices) {
            if (estop_device != nullptr) {
                estop_device->estop();
            }
        }
        this->estop_triggered = true;
        this->estop_triggered_time = millis();
    }

    void resume() {
        digitalWrite(MAIN_CONTACTOR_PIN, LOW);
        estop_triggered = false;
        estop_triggered_time = 0;
        estop_resume_time = millis();
        this->automatic_estop_inhibited = true;
    }

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(estop_sub);
    }

    void update_strings(){
        if (this->estop_triggered) {
            sprintf(this->diagnostic_topic->values[0].value, "True");
            this->diagnostic_topic->level = 2;
            sprintf(this->message_string, "E-Stop Triggered");
        } else if (!this->automatic_estop_enabled) {
            sprintf(this->diagnostic_topic->values[0].value, "False");
            this->diagnostic_topic->level = 1;
            sprintf(this->message_string, "Automatic E-Stop Disabled");
        } else {
            sprintf(this->diagnostic_topic->values[0].value, "False");
            this->diagnostic_topic->level = 0;
            sprintf(this->message_string, "OK");
        }
    }

    void update() override {
        this->update_strings();
        if (!estop_triggered) {
            if (millis() - estop_resume_time > 3000)
                this->automatic_estop_inhibited = false;
            this->check_for_faults();
        } else {
            if (millis() - estop_triggered_time > ESTOP_CONTACTOR_DELAY)
                digitalWriteFast(MAIN_CONTACTOR_PIN, HIGH);  // Open the main contactor
        }
    }

};


#endif //PRIMROSE_MCIU_ESTOPCONTROLLER_H
