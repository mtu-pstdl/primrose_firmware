//
// Created by Jay on 12/20/2022.
//

#include "ODriveROS.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/time.h"



/**
 * This method sets up the ROS publishers and subscribers
 */
void ODriveROS::subscribe(ros::NodeHandle *nh) {
    nh->subscribe(this->setpoint_sub);
}

/**
 * This function is called when a serial_message is received on the setpoint topic
 */
void ODriveROS::setpoint_callback(const std_msgs::Int32MultiArray &msg) {
    this->last_ros_command = millis();
    switch (static_cast<ODriveROS::ROS_COMMANDS>(msg.data[0])) {
        case E_STOP:
            this->odrive->emergency_stop();
            break;
        case DISARM:
            this->odrive->set_axis_state(odrive::axis_states::IDLE);
            break;
        case CLEAR_ERRORS:
            this->odrive->clear_errors();
            break;
        case SET_CLOSED_LOOP:
            // Check if we are already in the desired mode to avoid unnecessary can messages
            this->odrive->set_control_mode(
                    static_cast<odrive::control_modes>(msg.data[1]),
                    static_cast<odrive::input_modes>(msg.data[2]));
            break;
        case SET_POINT:
            this->odrive->set_setpoint(ODriveROS::from_fixed_point(msg.data[1], UNIT_SCALE));
            break;
        case CALIBRATE:
            this->odrive->calibrate();
    }
}

int32_t ODriveROS::to_fixed_point(double_t value, float scale) {
    return (int32_t)(value * scale);
}

int32_t ODriveROS::to_fixed_point(float value, float scale) {
    return (int32_t)(value * scale);
}

float_t ODriveROS::from_fixed_point(int32_t value, float scale) {
    return (float)value / scale;
}

void ODriveROS::update() {
    // Publish the condition topic
    this->output_topic->data[1] =  this->to_fixed_point(this->odrive->get_pos_estimate(), UNIT_SCALE);
    this->output_topic->data[2] =  this->to_fixed_point(this->odrive->get_vel_estimate(), UNIT_SCALE);
    this->output_topic->data[3] =  this->to_fixed_point(this->odrive->get_setpoint(), UNIT_SCALE);
    this->output_topic->data[4] =  this->odrive->get_control_mode();
    this->output_topic->data[5] =  this->odrive->get_axis_state();
    this->output_topic->data[6] =  static_cast<int32_t>(this->odrive->get_axis_error());
    this->output_topic->data[7] =  static_cast<int32_t>(this->odrive->get_active_errors());
    this->output_topic->data[8] =  static_cast<int32_t>(this->odrive->get_disarm_reason());
    this->output_topic->data[9] =  static_cast<int32_t>(this->odrive->get_procedure_results());
    this->output_topic->data[10] = this->to_fixed_point(this->odrive->get_torque_estimate(), UNIT_SCALE);
    this->output_topic->data[11] = this->to_fixed_point(this->odrive->get_vbus_voltage(), UNIT_SCALE);
    this->output_topic->data[12] = this->to_fixed_point(this->odrive->get_vbus_current(), UNIT_SCALE);
    this->output_topic->data[13] = this->to_fixed_point(this->odrive->get_odometer(), UNIT_SCALE);
    this->output_topic->data[14] = this->to_fixed_point(this->odrive->get_power_consumption(), UNIT_SCALE);
    this->output_topic->data[15] = this->to_fixed_point(this->odrive->get_Iq_measured(), UNIT_SCALE);
    this->output_topic->data[16] = this->to_fixed_point(this->odrive->get_Iq_setpoint(), UNIT_SCALE);

    if (odrive->get_axis_state() == odrive::axis_states::CLOSED_LOOP_CONTROL &&
       this->last_ros_command < millis() - 2500) {
        // If we haven't received a ROS command in 2.5 initiate an emergency stop
        this->odrive->emergency_stop();
    }
}

ODrivePro* ODriveROS::get_odrive() {
    return this->odrive;
}