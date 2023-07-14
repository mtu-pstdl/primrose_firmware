//
// Created by Jay on 3/20/2023.
//

#include "ActuatorsROS.h"

void ActuatorsROS::control_callback(const std_msgs::Int32MultiArray &msg) {
    // First element is the command and the second is the target actuator
    switch (msg.data[0]) {
        case STOP:
//            if (msg.data_length != 2) return;
            this->actuator->set_control_mode(ActuatorUnit::control_modes::E_STOPPED, msg.data[1]);
            break;
        case SET_POSITION:
//            if (msg.data_length != 3) return;
//            this->actuator->set_control_mode(static_cast<ActuatorUnit::control_modes>(msg.data[1]),
//                                             msg.data[2]);
            break;
        case SET_DUTY_CYCLE:
//            if (msg.data_length != 3) return;
//            this->actuator->set_target_position(msg.data[1], msg.data[2]);
            this->actuator->set_duty_cycle(msg.data[1] / 100.f, msg.data[2]);
            break;
    }
}

void ActuatorsROS::update_status_message(){
    if (this->actuator->connected) {
        if (this->actuator->get_temperature() > 80) {
            this->diagnostic_topic->level = 2;
            sprintf(this->status_string, "TEMP_CRITICAL: %3.1fC", this->actuator->get_temperature());
        } else if (this->actuator->get_temperature() > 65){
            this->diagnostic_topic->level = 1;
            sprintf(this->status_string, "TEMP_WARN: %3.1fC", this->actuator->get_temperature());
        } else {
            sprintf(this->status_string, "%24s", this->actuator->get_status_string());
            this->diagnostic_topic->level = 0;
        }
    } else {
        sprintf(this->status_string, "%24s", "Not Connected");
        this->diagnostic_topic->level = 2;
    }
}

void ActuatorsROS::update_diagnostics_topic(){

    if (this->actuator->connected) {
        sprintf(strings[0], "%24s", this->actuator->get_motor_fault_string(0));
        sprintf(strings[1], "%24s", this->actuator->get_motor_fault_string(1));
        if (this->actuator->get_position(0) == INT32_MIN) {
            sprintf(strings[2], "NULL Ticks");
        } else sprintf(strings[2], "%07ld Ticks", this->actuator->get_position(0));
        if (this->actuator->get_position(1) == INT32_MIN) {
            sprintf(strings[3], "NULL Ticks");
        } else sprintf(strings[3], "%07ld Ticks", this->actuator->get_position(1));
        if (this->actuator->get_velocity(0) == INT32_MIN) {
            sprintf(strings[4], "NULL Ticks/s");
        } else sprintf(strings[4], "%07ld Ticks/s", this->actuator->get_velocity(0));
        if (this->actuator->get_velocity(1) == INT32_MIN) {
            sprintf(strings[5], "NULL Ticks/s");
        } else sprintf(strings[5], "%07ld Ticks/s", this->actuator->get_velocity(1));
        sprintf(strings[6], "%04.1f C", this->actuator->get_temperature());
        if (this->actuator->get_current(0) == INT32_MIN) {
            sprintf(strings[7], "M1:   NULL | M2:    NULL");
        } else sprintf(strings[7], "M1:   %05.2fA | M2:    %05.2fA",
                       this->actuator->get_current(0), this->actuator->get_current(1));
        if (this->actuator->get_main_battery_voltage() == INT32_MIN ||
            this->actuator->get_logic_battery_voltage() == INT32_MIN) {
            sprintf(strings[8], "Main:   NULL | Logic:   NULL");
        } else sprintf(strings[8], "Main: %04.1fV | Logic: %04.1fV",
                       this->actuator->get_main_battery_voltage(), this->actuator->get_logic_battery_voltage());
    }
    this->update_status_message();
}

void ActuatorsROS::update() {
//    this->actuator->update();
    this->output_topic->data[0] = this->actuator->get_position(0);
    this->output_topic->data[1] = this->actuator->get_target_position(0);
    this->output_topic->data[2] = this->actuator->get_velocity(0);
    this->output_topic->data[3] = 0; // Placeholder for current control mode
    this->output_topic->data[4] = 0; // Odometry placeholder
    this->output_topic->data[5] = 0; // Power usage placeholder
    this->output_topic->data[6] = this->actuator->get_position(1);
    this->output_topic->data[7] = this->actuator->get_target_position(1);
    this->output_topic->data[8] = this->actuator->get_velocity(1);
    this->output_topic->data[9] = 0; // Placeholder for current control mode
    this->output_topic->data[10] = 0; // Odometry placeholder
    this->output_topic->data[11] = 0; // Power usage placeholder
    update_diagnostics_topic();
}

void ActuatorsROS::publish() {
}

void ActuatorsROS::subscribe(ros::NodeHandle *node_handle) {
    node_handle->subscribe(this->command_sub);
}
