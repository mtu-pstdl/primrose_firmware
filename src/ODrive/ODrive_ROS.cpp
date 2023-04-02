//
// Created by Jay on 12/20/2022.
//

#include "ODrive_ROS.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/time.h"


/**
 * This method sets up the ROS publishers and subscribers
 */
void ODrive_ROS::subscribe(ros::NodeHandle *nh) {

    nh->subscribe(this->setpoint_sub);
}

/**
 * This function is called when a message is received on the setpoint topic
 * @param msg The length of the message is 3 values: [0] = Position, [1] = Velocity, [2] = Torque
 */
void ODrive_ROS::setpoint_callback(const std_msgs::Float32MultiArray &msg) {
    if(msg.data_length == 3){
//        this->setpoint = msg.data[0];

    }
}

void ODrive_ROS::control_mode_callback(const std_msgs::Int32MultiArray &msg){

}

void ODrive_ROS::update_diagnostics_label(){
    if (this->odrive->is_connected()){
        switch(this->odrive->get_axis_state()) {
            case odrive::CLOSED_LOOP_CONTROL:
                this->state_topic->level = diagnostic_msgs::DiagnosticStatus::OK;
                sprintf(status_string, "Running: %10s", this->odrive->get_control_mode_string());
                break;
            case odrive::UNDEFINED:
            case odrive::IDLE:
                if (this->odrive->get_axis_error() != 0) {
                    this->state_topic->level = diagnostic_msgs::DiagnosticStatus::ERROR;
                    sprintf(status_string, "ERROR: %15s", this->odrive->get_axis_error_string());
                } else if (this->odrive->get_active_errors() != 0) {
                    this->state_topic->level = diagnostic_msgs::DiagnosticStatus::ERROR;
                    sprintf(status_string, "ERROR: %15s", this->odrive->get_active_errors_string());
                } else if (this->odrive->get_disarm_reason() != 0) {
                    this->state_topic->level = diagnostic_msgs::DiagnosticStatus::WARN;
                    sprintf(status_string, "FAULT: %15s", this->odrive->get_disarm_reason_string());
                } else if (this->odrive->get_procedure_results() == odrive::SUCCESS) {
                    this->state_topic->level = diagnostic_msgs::DiagnosticStatus::IDLE;
                    sprintf(status_string, "Ready");
                } else {
                    this->state_topic->level = diagnostic_msgs::DiagnosticStatus::WARN;
                    sprintf(status_string, "FAILED CALIB: %15s", this->odrive->get_procedure_results_string());
                }
                break;
            case odrive::STARTUP_SEQUENCE:
                this->state_topic->level = diagnostic_msgs::DiagnosticStatus::WARN;
                sprintf(status_string, "Starting");
                break;
            case odrive::FULL_CALIBRATION_SEQUENCE:
            case odrive::ENCODER_HALL_POLARITY_CALIBRATION:
            case odrive::ENCODER_INDEX_SEARCH:
            case odrive::ENCODER_OFFSET_CALIBRATION:
            case odrive::ENCODER_HALL_PHASE_CALIBRATION:
            case odrive::MOTOR_CALIBRATION:
                this->state_topic->level = diagnostic_msgs::DiagnosticStatus::OK;
                sprintf(status_string, "%s", this->odrive->get_axis_state_string());
                break;
            default:
                this->state_topic->level = diagnostic_msgs::DiagnosticStatus::ERROR;
                sprintf(status_string, "ERROR: %15s", this->odrive->get_axis_state_string());
                break;
        }
    } else {
        this->state_topic->level = diagnostic_msgs::DiagnosticStatus::ERROR;
        sprintf(status_string, "No Connection");
    }
}

void ODrive_ROS::update_diagnostics() {
    update_diagnostics_label();
    if (this->odrive->is_connected()) {
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
            sprintf(strings[2], "%lums", this->odrive->get_last_update()); // "Last Update:
            sprintf(strings[3], "%24s", this->odrive->get_setpoint_string());
            sprintf(strings[4], "%.2f %s", this->odrive->get_pos_estimate(), this->odrive->pos_unit_string);
            sprintf(strings[5], "%.2f %s", this->odrive->get_vel_estimate(), this->odrive->vel_unit_string);
        }
//        sprintf(strings[6], "%f C", this->odrive->get_fet_temp());
//        sprintf(strings[6], "%50s", this->odrive->get_fet_temp_frame_string());
        // Print the fet temp in hex
        sprintf(strings[6], "%.2f C", this->odrive->get_fet_temp());
        sprintf(strings[7], "%.2f C", this->odrive->get_motor_temp());
        sprintf(strings[8], "%.2f V", this->odrive->get_vbus_voltage());
        sprintf(strings[9], "%.2f A", this->odrive->get_vbus_current());
        sprintf(strings[10], "%.2f A", this->odrive->get_Iq_measured());
        sprintf(strings[11], "%.2f A", this->odrive->get_Iq_setpoint());
        // Show the binary representation of the inflight bitmask
        for (int i = 0; i < 7; i++) {
            if (this->odrive->get_inflight_bitmask() & (1 << i)) {
                strings[12][i] = '1';
            } else {
                strings[12][i] = '0';
            }
        }
    }
}

void ODrive_ROS::update_all() {
    // Publish the condition topic
    this->encoder_topic->data[0] = this->odrive->get_pos_estimate();
    this->encoder_topic->data[1] = this->odrive->get_vel_estimate();
    this->encoder_topic->data[2] = 0;
    this->encoder_topic->data[3] = this->odrive->get_control_mode();
    this->encoder_topic->data[4] = this->odrive->get_setpoint();
    update_diagnostics();
}

ODrivePro* ODrive_ROS::get_odrive() {
    return this->odrive;
}