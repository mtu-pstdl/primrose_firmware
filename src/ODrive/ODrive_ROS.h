//
// Created by Jay on 12/20/2022.
//

#ifndef TEENSYCANTRANSCEIVER_ODRIVE_ROS_H
#define TEENSYCANTRANSCEIVER_ODRIVE_ROS_H


#include "odrive_constants.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
//#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/publisher.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Float32MultiArray.h"
#include "ODriveS1.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/node_handle.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/UInt32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/publisher.h"

#define TOPIC_BASE "/mciu/ODrives/"

class ODrive_ROS {

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
    ros::Publisher condition_pub;

    // Publishes the values of POS_ESTIMATE, VEL_ESTIMATE, Iq_Setpoint, Iq_Measured
    std_msgs::Float32MultiArray encoder_topic;
    ros::Publisher encoder_pub;

    // Publishes the values of AXIS_STATE, AXIS_ERROR, ACTIVE_ERRORS, DISARM_REASON
    std_msgs::UInt32MultiArray state_topic;
    ros::Publisher state_pub;

    float_t setpoint = 0; // The setpoint of the ODrive

public:

    ODrive_ROS(ODriveS1* odrive, const char* random) :
            setpoint_sub(random,&ODrive_ROS::setpoint_callback, this),
            control_mode_sub(random,&ODrive_ROS::control_mode_callback, this),
            condition_pub(random, &condition_topic), encoder_pub(random, &encoder_topic),
            state_pub(random, &state_topic){
        this->odrive = odrive;
    }

    ODriveS1* get_odrive();

    void setpoint_callback(const std_msgs::Float32MultiArray &msg);

    void control_mode_callback(const std_msgs::Int32MultiArray &msg);

    void advertise(ros::NodeHandle* node_handle);

    void publish_all();
};


#endif //TEENSYCANTRANSCEIVER_ODRIVE_ROS_H
