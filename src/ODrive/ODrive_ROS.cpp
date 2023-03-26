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
//        this->setpoint = msg.data[0];
        if (this->control_mode == odrive::POSITION_CONTROL){
            // When in position control mode, the velocity and torque ff values are halfed to 16 bits
            // So we need to truncate the values of each float to 16 bits with a factor of 0.001

        }
    }
}

void ODrive_ROS::control_mode_callback(const std_msgs::Int32MultiArray &msg){

}

void ODrive_ROS::update_diagnostics() {
    if (!this->odrive->is_connected()) {
        if (this->odrive->get_axis_state() != odrive::CLOSED_LOOP_CONTROL) {
            update_diagnostics_keys(true);
            sprintf(strings[0], "%24s", this->odrive->get_axis_state_string());         // Axis State
            sprintf(strings[1], "%24s", this->odrive->get_axis_error_string());         // Axis Error
            sprintf(strings[2], "%24s", this->odrive->get_active_errors_string());      // Active Errors
            sprintf(strings[3], "%24s", this->odrive->get_disarm_reason_string());      // Disarm Reason
            sprintf(strings[4], "%24s", this->odrive->get_procedure_results_string());  // Procedure Results
            sprintf(strings[5], "%24s", this->odrive->get_control_mode_string());       // Control Mode
        } else {
            update_diagnostics_keys(false);
            sprintf(strings[0], "%24s", this->odrive->get_axis_state_string());
            sprintf(strings[1], "%24s", this->odrive->get_control_mode_string());
            sprintf(strings[2], "%.2f", this->odrive->get_setpoint());
            sprintf(strings[3], "%24s", this->odrive->get_disarm_reason_string());
            sprintf(strings[4], "%.2f", this->odrive->get_pos_estimate());
            sprintf(strings[5], "%.2f", this->odrive->get_vel_estimate());
        }
        sprintf(strings[6], "%2.2f C", this->odrive->get_fet_temp());
        sprintf(strings[7], "%2.2f C", this->odrive->get_motor_temp());
        sprintf(strings[8], "%2.2f V", this->odrive->get_vbus_voltage());
        sprintf(strings[9], "%2.2f A", this->odrive->get_vbus_current());
        sprintf(strings[10], "%2.2f A", this->odrive->get_Iq_measured());
        sprintf(strings[11], "%2.2f A", this->odrive->get_Iq_setpoint());
    } else {
        sprintf(status_string, "Not Connected!");
        this->state_topic->level = 2;
    }
}

void ODrive_ROS::publish_all() {
    // Publish the condition topic
    condition_topic.data[0] = this->odrive->get_fet_temp();
    condition_topic.data[1] = this->odrive->get_motor_temp();
    condition_topic.data[2] = this->odrive->get_vbus_voltage();
    condition_topic.data[3] = this->odrive->get_vbus_current();
    condition_topic.data[4] = this->odrive->get_Iq_measured();

    update_diagnostics();
}

ODriveS1* ODrive_ROS::get_odrive() {
    return this->odrive;
}