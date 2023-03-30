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
    this->actuator->set_target_position(msg.data[0], msg.data[1]);
}

void ActuatorsROS::begin_homing() {
    this->actuator->set_control_mode(ActuatorUnit::control_modes::homing, 0);
    this->actuator->set_control_mode(ActuatorUnit::control_modes::homing, 1);
}

void ActuatorsROS::update_status_message(){
    if (!this->actuator->connected){
        sprintf(this->status_string, "%24s", this->actuator->get_status_string());
        this->diagnostic_topic->level = 1;
    } else {
        sprintf(this->status_string, "%24s", "Not Connected");
    }
}

void ActuatorsROS::update_diagnostics_topic(){

    if (!this->actuator->connected) {
        sprintf(strings[0], "%24s", this->actuator->get_motor_fault_string(0));
        sprintf(strings[1], "%24s", this->actuator->get_motor_fault_string(1));
        sprintf(strings[2], "%ld Ticks", this->actuator->get_position(0));
        sprintf(strings[3], "%ld Ticks", this->actuator->get_position(1));
        sprintf(strings[4], "%ld Ticks/s", this->actuator->get_velocity(0));
        sprintf(strings[5], "%ld Ticks/s", this->actuator->get_velocity(1));
        sprintf(strings[6], "%2.1f C", this->actuator->get_temperature());
        sprintf(strings[7], "M1:   %2.1fA | M2:    %2.1fA",
                this->actuator->get_current(0), this->actuator->get_current(1));
        sprintf(strings[8], "Main: %2.1fV | Logic: %2.1fV",
                this->actuator->get_main_battery_voltage(), this->actuator->get_logic_battery_voltage());
    }
    this->update_status_message();
}

void ActuatorsROS::update() {
    this->actuator->update();
    update_diagnostics_topic();
}

void ActuatorsROS::publish() {
    encoder_pub_.publish(&encoder_topic);
//    state_pub_.publish(&state_topic);
}
