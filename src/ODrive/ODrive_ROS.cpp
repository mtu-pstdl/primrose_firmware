//
// Created by Jay on 12/20/2022.
//

#include "ODrive_ROS.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"


/**
 * This method sets up the ROS publishers and subscribers
 */
void ODrive_ROS::advertise() {
    this->node_handle->advertise(condition_pub);
    this->node_handle->advertise(encoder_pub);
    this->node_handle->advertise(state_pub);
    this->node_handle->subscribe(setpoint_sub);
    this->node_handle->subscribe(control_mode_sub);
}

/**
 * This function is called when a message is received on the setpoint topic
 * @param msg The length of the message is 3 values: [0] = Position, [1] = Velocity, [2] = Torque
 */
void ODrive_ROS::setpoint_callback(const std_msgs::Float32MultiArray &msg) {
    if(msg.data_length == 3){
        this->setpoint = msg.data[0];
        if (this->control_mode == odrive::POSITION_CONTROL){
            // When in position control mode, the velocity and torque ff values are halfed to 16 bits
            // So we need to truncate the values of each float to 16 bits with a factor of 0.001

        }
    }
}

void ODrive_ROS::control_mode_callback(const std_msgs::Int32MultiArray &msg){

}


void ODrive_ROS::publish_all() {

}


ODrive_ROS::ODrive_ROS(const String& name, ros::NodeHandle* nh, int axis_number,
                       FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> *can_bus) :
        condition_pub(String(TOPIC_BASE + name + "/condition").c_str(), &condition_topic),
        encoder_pub(String(TOPIC_BASE + name + "/encoder").c_str(), &encoder_topic),
        state_pub(String(TOPIC_BASE + name + "/state").c_str(), &state_topic),
        setpoint_sub(String(TOPIC_BASE + name + "/setpoint").c_str(),
                     &ODrive_ROS::setpoint_callback, this),
        control_mode_sub(String(TOPIC_BASE + name + "/control_mode").c_str(),
                         &ODrive_ROS::control_mode_callback, this) {
        this->odrive = new ODriveS1(axis_number, name, can_bus);
    }