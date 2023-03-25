//
// Created by Jay on 12/20/2022.
//

#include "ODrive_ROS.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"


/**
 * This method sets up the ROS publishers and subscribers
 */
void ODrive_ROS::advertise_subscribe(ros::NodeHandle *nh) {
    this->node_handle = nh;

    nh->advertise(this->condition_pub_);
    nh->advertise(this->encoder_pub_);
//    nh->advertise(this->state_pub_);
//    nh->subscribe(this->setpoint_sub);
//    nh->subscribe(this->control_mode_sub);

    String log = "Advertised topics for " + *this->odrive->name;
    nh->loginfo(log.c_str());
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
//    this->condition_pub_.publish(&condition_topic);

    // Publish the encoder topic

//    encoder_topic.data[0] = this->odrive->get_pos_estimate();
//    encoder_topic.data[1] = this->odrive->get_vel_estimate();
//    encoder_topic.data[2] = this->odrive->get_Iq_setpoint();
//    encoder_topic.data[4] = this->odrive->get_setpoint();
//    this->encoder_pub_.publish(&encoder_topic);

    // Publish the state topic
    if (this->odrive->is_connected()){
        this->state_topic->level = 0;
        this->state_topic->message = "Connected";
        sprintf(strings[0], "%s", this->odrive->get_axis_state_string());
        sprintf(strings[1], "%s", this->odrive->get_axis_error_string());
        sprintf(strings[2], "%s", this->odrive->get_active_errors_string());
        sprintf(strings[3], "%s", this->odrive->get_disarm_reason_string());
        sprintf(strings[4], "%2.2f", this->odrive->get_fet_temp());
        sprintf(strings[5], "%2.2f", this->odrive->get_motor_temp());
        sprintf(strings[6], "%2.2f", this->odrive->get_vbus_voltage());
        sprintf(strings[7], "%2.2f", this->odrive->get_vbus_current());
        sprintf(strings[8], "%2.2f", this->odrive->get_pos_estimate());
        sprintf(strings[9], "%2.2f", this->odrive->get_vel_estimate());
        sprintf(strings[10], "%2.2f", this->odrive->get_Iq_measured());
    } else {
        this->state_topic->level = 2;
        this->state_topic->message = "Not Connected!";
    }

}

ODriveS1* ODrive_ROS::get_odrive() {
    return this->odrive;
}