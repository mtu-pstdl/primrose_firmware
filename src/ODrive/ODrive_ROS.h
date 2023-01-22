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

#define TOPIC_BASE "/mciu/ODrives/"

class ODrive_ROS {

    ODriveS1 *odrive = nullptr;

    ros::Subscriber<std_msgs::Float32MultiArray, ODrive_ROS> setpoint_sub;
    ros::Subscriber<std_msgs::Int32MultiArray, ODrive_ROS> control_mode_sub;

    odrive::control_modes control_mode = odrive::control_modes::UNKNOWN_CONTROL_MODE; // The control mode of the ODrive
    odrive::input_modes input_mode = odrive::input_modes::UNKNOWN_INPUT_MODE; // The input mode of the ODrive

    ros::NodeHandle* node_handle = nullptr; // The ROS node handle

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

    ODrive_ROS(const String& name, ros::NodeHandle* nh, int axis_number, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> *can_bus);

    void setpoint_callback(const std_msgs::Float32MultiArray &msg);

    void control_mode_callback(const std_msgs::Int32MultiArray &msg);

    void advertise();

    void publish_all();
};


#endif //TEENSYCANTRANSCEIVER_ODRIVE_ROS_H
