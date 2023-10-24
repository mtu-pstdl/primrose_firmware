//
// Created by Jay on 3/20/2023.
//

#include "ActuatorsROS.h"

void ActuatorsROS::control_callback(const std_msgs::Int32MultiArray &msg) {
    // First element is the command and the second is the target actuator
    switch (msg.data[0]) {
        case STOP:
            this->actuator->set_duty_cycle(0, msg.data[1]);
            break;
        case SET_POSITION:
            if (msg.data_length != 3) return;
            this->actuator->set_target_position(msg.data[1], msg.data[2]);
            break;
        case SET_DUTY_CYCLE:
            if (msg.data_length != 3) return;
            this->actuator->set_duty_cycle(msg.data[1] / 100.f, msg.data[2]);
            break;
    }
}

void ActuatorsROS::update() {
//    this->actuator->update();
    this->output_topic->data[1] = this->actuator->get_position(0);
    this->output_topic->data[2] = 0; // Velocity steering
    this->output_topic->data[3] = this->actuator->get_position(1);
    this->output_topic->data[4] = 0; // Velocity

    this->output_topic->data[5] = this->actuator->get_duty_cycle(0) * 100;
    this->output_topic->data[6] = this->actuator->get_duty_cycle(1) * 100;
}

void ActuatorsROS::publish() {
}

void ActuatorsROS::subscribe(ros::NodeHandle *node_handle) {
    node_handle->subscribe(this->command_sub);
}
