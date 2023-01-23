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
    // Publish the condition topic
    condition_topic.data[0] = this->odrive->get_fet_temp();
    condition_topic.data[1] = this->odrive->get_motor_temp();
    condition_topic.data[2] = this->odrive->get_vbus_voltage();
    condition_topic.data[3] = this->odrive->get_vbus_current();
    condition_topic.data[4] = this->odrive->get_Iq_measured();
    condition_pub.publish(&condition_topic);

    // Publish the encoder topic
    encoder_topic.data[0] = this->odrive->get_pos_estimate();
    encoder_topic.data[1] = this->odrive->get_vel_estimate();
    encoder_topic.data[2] = this->odrive->get_Iq_setpoint();
    encoder_topic.data[4] = this->odrive->get_setpoint();
    encoder_pub.publish(&encoder_topic);

    // Publish the state topic
    state_topic.data[0] = this->odrive->get_axis_state();
    state_topic.data[1] = this->odrive->get_axis_error();
    state_topic.data[2] = this->odrive->get_active_errors();
    state_topic.data[3] = this->odrive->get_disarm_reason();
    state_pub.publish(&state_topic);
}


ODrive_ROS::ODrive_ROS(const String& name, ros::NodeHandle* nh, ODriveS1* odrive) :
        condition_pub(String(TOPIC_BASE + name + "/condition").c_str(), &condition_topic),
        encoder_pub(String(TOPIC_BASE + name + "/encoder").c_str(), &encoder_topic),
        state_pub(String(TOPIC_BASE + name + "/state").c_str(), &state_topic),
        setpoint_sub(String(TOPIC_BASE + name + "/setpoint").c_str(),
                     &ODrive_ROS::setpoint_callback, this),
        control_mode_sub(String(TOPIC_BASE + name + "/control_mode").c_str(),
                         &ODrive_ROS::control_mode_callback, this) {
        this->odrive = odrive;
        this->node_handle = nh;
    }