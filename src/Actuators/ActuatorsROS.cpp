//
// Created by Jay on 3/20/2023.
//

#include "ActuatorsROS.h"

void ActuatorsROS::advertise_subscribe(ros::NodeHandle *nh) {
    node_handle = nh;
//    nh->advertise(encoder_pub_);
//    nh->advertise(state_pub_);
//    nh->subscribe(setpoint_sub);
//    nh->subscribe(control_mode_sub);
}

void ActuatorsROS::setpoint_callback(const std_msgs::Float32MultiArray &msg) {

}

void ActuatorsROS::control_mode_callback(const std_msgs::Int32MultiArray &msg) {

}

void ActuatorsROS::update() {
    this->actuator->update();
    if (!this->actuator->connected) {
        if (diagnostic_topic->level != 2) {
            String log_msg = "Actuator " + this->name + " went offline!";
            node_handle->loginfo(log_msg.c_str());
        }
        diagnostic_topic->level = 2;
        diagnostic_topic->message = "Not Connected!";
        diagnostic_topic->values[5].value = "Not Connected!";
    } else {
        diagnostic_topic->level = 0;
        diagnostic_topic->values[0].value = "0C";  // Temperature
        diagnostic_topic->values[1].value = "0A";  // Current
        diagnostic_topic->values[2].value = "0A";  // Current
        diagnostic_topic->values[3].value = "0V";  // Main Volts
        diagnostic_topic->values[4].value = "0V";  // Logic Volts
        diagnostic_topic->values[5].value = "Unknown";  // Status
    }
}

void ActuatorsROS::publish() {
    encoder_pub_.publish(&encoder_topic);
    state_pub_.publish(&state_topic);
}
