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
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/String.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"

#define MAIN_CONTACTOR_PIN 0

// The time in milliseconds to wait after an estop is triggered before the main contactor is opened (to reduce back EMF)
#define ESTOP_CONTACTOR_DELAY 2500  // 2.5 seconds
#define HEARTBEAT_INTERVAL 4000  // 4 seconds
#define STATUS_MESSAGE_LENGTH 1024

class EStopController : public ROSNode {

private:

    enum ros_commands {
        ESTOP = 0,
        RESUME = 1,
        ENABLE_AUTOMATIC = 2,
        DISABLE_AUTOMATIC = 3,
        PI_HEARTBEAT = 4,
        REMOTE_HEARTBEAT = 5,
    };

    ros::Subscriber<std_msgs::Int32, EStopController> estop_sub;
    std_msgs::String*          estop_msg_topic;
    std_msgs::Int32MultiArray* estop_status_topic;
//    diagnostic_msgs::DiagnosticStatus* diagnostic_topic;

    void estop_callback(const std_msgs::Int32& msg);

    /**
     * @brief A linked list of EStopDevices so that they can be added at runtime without knowing the number of devices
     *        at compile time
     */
    struct EStopDeviceList {
        EStopDevice*     estop_device = nullptr;
        boolean          has_tripped  = false;
        EStopDeviceList* next         = nullptr;
    };
    EStopDeviceList* estop_devices = new EStopDeviceList;

    void add_to_estop_device_list(EStopDevice* estop_device) {
        EStopDeviceList* current = estop_devices;
        while (true) {
            if (current->estop_device == nullptr) {
                current->estop_device = estop_device;
                return;
            }
            if (current->next == nullptr) break;
            current = current->next;
        }
        current->next = new EStopDeviceList;
        current->next->estop_device = estop_device;
    }

    /**
     * @brief Get an EStopDevice from the EStopDeviceList
     * @param index The index of the EStopDevice to get
     * @return A pointer to the EStopDevice at the given index or nullptr if the index is out of bounds
     */
    EStopDevice* get_estop_device(int32_t index) {
        EStopDeviceList* current = estop_devices;
        while (index > 0) {
            if (current->next == nullptr) return nullptr;
            current = current->next;
            index--;
        }
        return current->estop_device;
    }

    // Automatic E-Stop variables
    boolean         automatic_estop_enabled = true;
    boolean         automatic_estop_inhibited = false;
    boolean         should_trigger_estop = false;
    uint32_t        number_of_tripped_devices = 0;
    char*           tripped_device_name = new char[30];
    char*           tripped_device_message = new char[100];
    char*           estop_message = new char[STATUS_MESSAGE_LENGTH];

    // E-Stop variables
    boolean  estop_triggered = false;
    uint32_t estop_triggered_time = 0;
    uint32_t estop_resume_time = 0;  // The time to wait after the contactor is closed before the E-Stop is cleared
    uint32_t last_pi_heartbeat     = HEARTBEAT_INTERVAL;
    uint32_t last_remote_heartbeat = HEARTBEAT_INTERVAL;


    void check_for_faults();

    void pi_heartbeat() {
        this->last_pi_heartbeat = millis();
    }

    void remote_heartbeat() {
        this->last_remote_heartbeat = millis();
    }

public:

    /**
     * @brief Construct a new EStopController object (There should only be one of these)
     * @param estop_msg_topic A string message topic to publishes the fault messages from all tripped devices
     * @param estop_status_topic An Int32MultiArray topic to publish the status of the E-Stop controller
     */
    explicit EStopController(std_msgs::String* estop_msg_topic, std_msgs::Int32MultiArray *estop_status_topic) :
        estop_sub("/mciu/Estop_controller/input", &EStopController::estop_callback, this) {
        this->estop_msg_topic = estop_msg_topic;
        this->estop_status_topic = estop_status_topic;

        this->estop_msg_topic->data = this->estop_message;
        sprintf(this->estop_message, "All ok");

        this->estop_status_topic->data_length = 4;
        this->estop_status_topic->data = new int32_t[4];
        this->estop_status_topic->data[0] = 0;  // Current state
        this->estop_status_topic->data[1] = 0;  // Flags
        this->estop_status_topic->data[2] = 0;  // Number of tripped devices
        this->estop_status_topic->data[3] = 0;  // Time since last heartbeat

        sprintf(this->tripped_device_name, "NULL");
        sprintf(this->tripped_device_message, "NULL");

        pinMode(MAIN_CONTACTOR_PIN, OUTPUT);
        digitalWrite(MAIN_CONTACTOR_PIN, HIGH);
//        this->trigger_estop();

    }

    /**
     * @brief Add an EStopDevice to the EStopController
     * @param estop_device A pointer to the EStopDevice to add
     */
    void add_estop_device(EStopDevice* estop_device) {
        this->add_to_estop_device_list(estop_device);
    }

    void update() override;

    void publish() override {
    }

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(estop_sub);
    }

    bool is_high_voltage_enabled();

    void trigger_estop(boolean automatic = false, boolean remote = false);

    void resume();

};


#endif //PRIMROSE_MCIU_ESTOPCONTROLLER_H
