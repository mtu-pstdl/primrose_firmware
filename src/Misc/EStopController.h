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
#include "Main_Helpers/BreadCrumbs.h"
#include <TimeLib.h>

#define MAIN_CONTACTOR_PIN 5

// The time in milliseconds to wait after an estop is triggered before the main contactor is opened (to reduce back EMF)
#define ESTOP_CONTACTOR_DELAY 2500  // 2.5 seconds
#define HEARTBEAT_INTERVAL 4000  // 4 seconds
#define STATUS_MESSAGE_LENGTH 900

// EStopFlags is a bitfield of the current state of the E-Stop system
#define ESTOP_TRIGGERED         0x00000001  // An E-Stop has been triggered
#define AUTO_ESTOP_INHIBITED    0x00000002  // Automatic E-Stops are inhibited (e.g. An E-Stop is being cleared)
#define AUTO_ESTOP_ENABLED      0x00000004  // Automatic E-Stops are enabled (Can be enabled and inhibited at the same time)
#define HIGH_VOLTAGE_ENABLED    0x00000008  // The command to enable high voltage has been sent to the contactor
#define REMOTE_HEARTBEAT_LOW    0x00000010  // The remote heartbeat is about to expire (less than 1 second left)
#define PI_HEARTBEAT_LOW        0x00000020  // The PI heartbeat is about to expire     (less than 1 second left)
#define SOME_DEVICES_SUPPRESSED 0x00000040  // Some devices are not allowed to trigger an E-Stop

// Prefix characters for the E-Stop message (wip)
#define TRIGGERING_FAULT '-'
#define SUPPRESSED_FAULT '*'
#define WARNING_MESSAGE  '!'

/**
 * The EStopController class is responsible for managing all E-Stop devices and triggering E-Stops.
 * It also publishes the status of the E-Stop system to the ROS network.
 * @note There should only be one EStopController object and all EStopDevices should be attached to it.
 */
class EStopController : public ROSNode, public EStopDevice {

private:

    enum ros_commands {
        ESTOP             = 0,  // Trigger an E-Stop
        RESUME            = 1,  // Resume from an E-Stop
        ENABLE_AUTOMATIC  = 2,  // Enable automatic E-Stop
        DISABLE_AUTOMATIC = 3,  // Disable automatic E-Stop
        SUPPRESS_CURRENT  = 4,  // Ignore errors from currently tripped devices for future E-Stops (doesn't persist across restarts)
        PI_HEARTBEAT      = 5,  // A heartbeat from the PI to indicate that it is still running
        REMOTE_HEARTBEAT  = 6,  // A heartbeat from the drivers station to indicate that it is still running
    };

    ros::Subscriber<std_msgs::Int32, EStopController> estop_sub;
    std_msgs::String*          estop_msg_topic;
    std_msgs::Int32MultiArray* estop_status_topic;

    /**
     * EStopController::OutputArray is a named union for storing the output data into the Int32MultiArray message
     */
    union OutputArray {
        struct OutputData {
            int32_t estop_flags;                        // A bitfield of EStopFlags
            int32_t number_of_tripped_devices;
            int32_t time_since_last_pi_heartbeat;
            int32_t time_since_last_remote_heartbeat;
        } data;
        int32_t raw_array[4];  // The raw array of data to be sent over the serial bus
    } output_data = {};

    void estop_callback(const std_msgs::Int32& msg);

    /**
     * @brief A linked list of EStopDevices so that they can be added at runtime without knowing the number of devices
     *        at compile time
     */
    struct EStopDeviceList {
        EStopDevice*     estop_device = nullptr;
        boolean          suppressed   = false;
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
        current->next->suppressed = false;
        current->next->estop_device = estop_device;
        current->next->next = nullptr;
    }

    /**
     * @brief Get an EStopDevice from the EStopDeviceList
     * @param index The index of the EStopDevice to get
     * @return A pointer to the EStopDevice at the given index or nullptr if the index is out of bounds
     */
    EStopDeviceList* get_estop_device(int32_t index) {
        EStopDeviceList* current = estop_devices;
        while (index > 0) {
            if (current->next == nullptr) return nullptr;
            current = current->next;
            index--;
        }
        return current;
    }

    // Automatic E-Stop variables
    boolean         automatic_estop_enabled = false;
    boolean         automatic_estop_inhibited = false;
    boolean         should_trigger_estop = false;
    uint32_t        number_of_tripped_devices = 0;
    uint32_t        system_loops = 0;
    char*           tripped_device_name = new char[30];
    char*           tripped_device_message = new char[100];
    char*           estop_message = new char[STATUS_MESSAGE_LENGTH];
    char*           last_estop_message = new char[STATUS_MESSAGE_LENGTH];

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

    void suppress_current() {
        EStopDeviceList* current = estop_devices;
        char temp[150];
        while (current != nullptr) {
            if (current->estop_device != nullptr) {
                if (current->estop_device->tripped(temp, temp)) {
                    current->suppressed = true;
                } current->suppressed = false;
            }
            if (current->next == nullptr) break;
            current = current->next;
        }
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

        this->estop_msg_topic->data = this->last_estop_message;

        this->estop_status_topic->data_length = sizeof (this->output_data.raw_array) / sizeof (int32_t);
        this->estop_status_topic->data = this->output_data.raw_array;

        this->output_data.data.estop_flags = 0;
        this->output_data.data.number_of_tripped_devices = 0;
        this->output_data.data.time_since_last_pi_heartbeat = 0;
        this->output_data.data.time_since_last_remote_heartbeat = 0;

        sprintf(this->tripped_device_name, "NULL");
        sprintf(this->tripped_device_message, "NULL");

        this->add_to_estop_device_list(this);

        pinMode(MAIN_CONTACTOR_PIN, OUTPUT);
        digitalWrite(MAIN_CONTACTOR_PIN, HIGH);
//        sprintf(this->last_estop_message, "[E-Stop Controller Initializing]");
//        sprintf(this->estop_message, "[Firmware Start - Main Contactor Opened - E-Stop Triggered]");

//        this->trigger_estop(false, false);
    }

    /**
     * @brief Add an EStopDevice to the EStopController
     * @param estop_device A pointer to the EStopDevice to add
     */
    void add_estop_device(EStopDevice* estop_device) {
        this->add_to_estop_device_list(estop_device);
    }

    void update() override;

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(estop_sub);
    }

    boolean estop_message_updated(){
        DROP_CRUMB();
        this->system_loops++;
        if (this->system_loops % 100 == 0) {
            return true;
        }
        if (strcmp(this->estop_message, this->last_estop_message) != 0) {
            sprintf(this->last_estop_message, "%s", this->estop_message);
            return true;
        } return false;
    }

    bool is_high_voltage_enabled();

    void trigger_estop(boolean automatic = false, boolean remote = false);

    void resume();

    TRIP_LEVEL tripped(char* name, char* message) override;

};


#endif //PRIMROSE_MCIU_ESTOPCONTROLLER_H
