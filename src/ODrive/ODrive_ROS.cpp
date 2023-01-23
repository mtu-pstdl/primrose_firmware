//
// Created by Jay on 12/20/2022.
//

#include "ODrive_ROS.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"


/**
 * This method sets up the ROS publishers and subscribers
 */
void ODrive_ROS::advertise(ros::NodeHandle *nh) {
    this->node_handle = nh;
    // Set the name of the topics
    String condition_topic_name = TOPIC_BASE + *this->odrive->name + "/condition";
    String encoder_topic_name = TOPIC_BASE + *this->odrive->name + "/encoder";
    String state_topic_name = TOPIC_BASE + *this->odrive->name + "/state";
    String setpoint_topic_name = TOPIC_BASE + *this->odrive->name + "/setpoint";
    String control_mode_topic_name = TOPIC_BASE + *this->odrive->name + "/control_mode";

    // Overwrite the topic names
    condition_pub.topic_ = condition_topic_name.c_str();
    encoder_pub.topic_ = encoder_topic_name.c_str();
    state_pub.topic_ = state_topic_name.c_str();
    setpoint_sub.topic_ = setpoint_topic_name.c_str();
    control_mode_sub.topic_ = control_mode_topic_name.c_str();

    nh->advertise(condition_pub);
    nh->advertise(encoder_pub);
    nh->advertise(state_pub);
    nh->subscribe(setpoint_sub);
    nh->subscribe(control_mode_sub);
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

ODriveS1* ODrive_ROS::get_odrive() {
    return this->odrive;
}
