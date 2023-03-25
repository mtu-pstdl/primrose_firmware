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
        strings[0]->replace(strings[0]->length(), String(this->odrive->get_pos_estimate()));
        strings[1]->replace(strings[1]->length(), String(this->odrive->get_vel_estimate()));
        strings[2]->replace(strings[2]->length(), String(this->odrive->get_Iq_setpoint()));
        strings[3]->replace(strings[3]->length(), String(this->odrive->get_setpoint()));
    } else {
        this->state_topic->level = 2;
        this->state_topic->message = "Not Connected!";
    }


    state_topic->values[0].value = strings[0]->c_str();
    state_topic->values[1].value = strings[1]->c_str();
    state_topic->values[2].value = strings[2]->c_str();
    state_topic->values[3].value = strings[3]->c_str();
}

ODriveS1* ODrive_ROS::get_odrive() {
    return this->odrive;
}