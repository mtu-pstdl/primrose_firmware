//
// Created by Jay on 3/20/2023.
//

#include "ActuatorsROS.h"
#include "Main_Helpers/BreadCrumbs.h"

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

int32_t ActuatorsROS::to_fixed_point(float value, float scale) {
    // If the value is nan or inf, return 0
    if (isnan(value) || isinf(value)) return INT32_MIN;
    return (int32_t)(value * scale);
}

void ActuatorsROS::update() {
    DROP_CRUMB();
    this->actuator->update();

    this->output_data.data.m1_position = this->actuator->get_position(0);
    this->output_data.data.m1_velocity = this->to_fixed_point(this->actuator->get_velocity(0), UNIT_SCALE);
    this->output_data.data.m2_position = this->actuator->get_position(1);
    this->output_data.data.m2_velocity = this->to_fixed_point(this->actuator->get_velocity(1), UNIT_SCALE);

    // End of data used by LaRP controller, remaining data is for diagnostics
    this->output_data.data.m1_duty_cycle = this->to_fixed_point(this->actuator->get_duty_cycle(0), UNIT_SCALE);
    this->output_data.data.m2_duty_cycle = this->to_fixed_point(this->actuator->get_duty_cycle(1), UNIT_SCALE);

    // Controller information
    this->output_data.data.m1_target_position = this->actuator->get_target_position(0);
    this->output_data.data.m2_target_position = this->actuator->get_target_position(1);

    // Get the current mode
    this->output_data.data.m1_control_mode = this->actuator->get_control_mode(0);
    this->output_data.data.m2_control_mode = this->actuator->get_control_mode(1);

    // Current information
    this->output_data.data.m1_current = this->to_fixed_point(this->actuator->get_current(0), UNIT_SCALE);
    this->output_data.data.m2_current = this->to_fixed_point(this->actuator->get_current(1), UNIT_SCALE);

    // Fault information
    this->output_data.data.m1_fault_flags = this->actuator->get_fault_flags(0);
    this->output_data.data.m2_fault_flags = this->actuator->get_fault_flags(1);

    // Power information
    this->output_data.data.main_battery_voltage = this->to_fixed_point(this->actuator->get_main_battery_voltage(), UNIT_SCALE);
    this->output_data.data.logic_battery_voltage = this->to_fixed_point(this->actuator->get_logic_battery_voltage(), UNIT_SCALE);

    // Temperature information
    this->output_data.data.controller_temperature = this->to_fixed_point(this->actuator->get_temperature(), UNIT_SCALE);
}

void ActuatorsROS::publish() {
}

void ActuatorsROS::subscribe(ros::NodeHandle *node_handle) {
    node_handle->subscribe(this->command_sub);
}
