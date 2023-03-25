//
// Created by Jay on 12/20/2022.
//

#ifndef TEENSYCANTRANSCEIVER_ODRIVE_ROS_H
#define TEENSYCANTRANSCEIVER_ODRIVE_ROS_H


#include "odrive_constants.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/node_handle.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/publisher.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/service_server.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Float32MultiArray.h"
#include "ODriveS1.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/UInt32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Empty.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/KeyValue.h"

#define TOPIC_BASE "/mciu/ODrives"

class ODrive_ROS {

    const char* topic_names[6][6] = {
        {TOPIC_BASE "/x00/condition", TOPIC_BASE "/x01/condition", TOPIC_BASE "/x02/condition",
                TOPIC_BASE "/x03/condition", TOPIC_BASE "/x04/condition", TOPIC_BASE "/x05/condition"},
        {TOPIC_BASE "/x00/encoder", TOPIC_BASE "/x01/encoder", TOPIC_BASE "/x02/encoder",
                TOPIC_BASE "/x03/encoder", TOPIC_BASE "/x04/encoder", TOPIC_BASE "/x05/encoder"},
        {TOPIC_BASE "/x00/state", TOPIC_BASE "/x01/state", TOPIC_BASE "/x02/state",
                TOPIC_BASE "/x03/state", TOPIC_BASE "/x04/state", TOPIC_BASE "/x05/state"},
        {TOPIC_BASE "/x00/setpoint", TOPIC_BASE "/x01/setpoint", TOPIC_BASE "/x02/setpoint",
                TOPIC_BASE "/x03/setpoint", TOPIC_BASE "/x04/setpoint", TOPIC_BASE "/x05/setpoint"},
        {TOPIC_BASE "/x00/control_mode", TOPIC_BASE "/x01/control_mode", TOPIC_BASE "/x02/control_mode",
                TOPIC_BASE "/x03/control_mode", TOPIC_BASE "/x04/control_mode", TOPIC_BASE "/x05/control_mode"},
    };


    ODriveS1 *odrive = nullptr;
    String name = ""; // The name of the ODrive

private:
    ros::NodeHandle* node_handle = nullptr; // The ROS node handle

    ros::Subscriber<std_msgs::Float32MultiArray, ODrive_ROS> setpoint_sub;
    ros::Subscriber<std_msgs::Int32MultiArray, ODrive_ROS> control_mode_sub;

    odrive::control_modes control_mode = odrive::control_modes::UNKNOWN_CONTROL_MODE; // The control mode of the ODrive
    odrive::input_modes input_mode = odrive::input_modes::UNKNOWN_INPUT_MODE; // The input mode of the ODrive

    // Publishes the values of FET_TEMP, MOTOR_TEMP, VBUS_VOLTAGE, VBUS_CURRENT
    std_msgs::Float32MultiArray condition_topic;

    // Publishes the values of POS_ESTIMATE, VEL_ESTIMATE, Iq_Setpoint, Iq_Measured
    std_msgs::Float32MultiArray encoder_topic;

    // Publishes the values of AXIS_STATE, AXIS_ERROR, ACTIVE_ERRORS, DISARM_REASON
    diagnostic_msgs::DiagnosticStatus* state_topic;

    char* strings[11];

    void allocate_strings() {
        for (auto & string : strings) {
            string = new char[25];
        }
    }

    String* unknown_string = new String("Unknown");

    // Setup service server

    float_t setpoint = 0; // The setpoint of the ODrive

public:

    ros::Publisher condition_pub_;
    ros::Publisher encoder_pub_;
//    ros::Publisher state_pub_;

    ODrive_ROS(ODriveS1* odrive, uint8_t number, diagnostic_msgs::DiagnosticStatus* status, String disp_name) :
            setpoint_sub(topic_names[3][number], &ODrive_ROS::setpoint_callback, this),
            control_mode_sub(topic_names[4][number], &ODrive_ROS::control_mode_callback, this),
            condition_pub_("condition", &condition_topic),
            encoder_pub_("encoder", &encoder_topic) {
        this->odrive = odrive;
        this->condition_topic.data_length = 5;
        this->condition_topic.data = new float_t[5];
        this->encoder_topic.data_length = 5;
        this->encoder_topic.data = new float_t[5];
        this->state_topic = status;

        this->state_topic->values_length = 11;
        this->state_topic->values = new diagnostic_msgs::KeyValue[11];
        state_topic->values[0].key = "AXIS_STATE";
        state_topic->values[1].key = "AXIS_ERROR";
        state_topic->values[2].key = "ACTIVE_ERRORS";
        state_topic->values[3].key = "DISARM_REASON";
        state_topic->values[4].key = "FET_TEMP";
        state_topic->values[5].key = "MOTOR_TEMP";
        state_topic->values[6].key = "VBUS_VOLTAGE";
        state_topic->values[7].key = "VBUS_CURRENT";
        state_topic->values[8].key = "POS_ESTIMATE";
        state_topic->values[9].key = "VEL_ESTIMATE";
        state_topic->values[10].key = "Iq_Setpoint";
        this->name = disp_name.c_str();
        state_topic->name = "ODrive";
        state_topic->message = "Initializing";
        state_topic->level = 0;
        state_topic->hardware_id = this->name.c_str();

        allocate_strings();
        for (int i = 0; i < 11; i++) {
            this->state_topic->values[i].value = strings[i];
        }
    }


    ODriveS1* get_odrive();

    void setpoint_callback(const std_msgs::Float32MultiArray &msg);

    void control_mode_callback(const std_msgs::Int32MultiArray &msg);

    void advertise_subscribe(ros::NodeHandle* node_handle);

    void publish_all();
};


#endif //TEENSYCANTRANSCEIVER_ODRIVE_ROS_H
