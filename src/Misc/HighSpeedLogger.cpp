//
// Created by Jay on 3/8/2024.
//

#include "HighSpeedLogger.h"

void HighSpeedLogger::attach_to_odrive(ODrivePro *target_odrive) {
    this->odrive = target_odrive;
    this->odrive->enable_high_frequency_logging(this);
}

void HighSpeedLogger::detach_from_odrive() {

}

uint64_t HighSpeedLogger::get_micros_64() {
    return micros();
}

HighSpeedLogger::LogData HighSpeedLogger::get_log_data(ODrivePro* target, odrive::command_ids data_type){
    HighSpeedLogger::LogData log_data = {};
    log_data.command_id = data_type;
    // Get microseconds in 64 bit resolution
    uint64_t micros = get_micros_64();
    log_data.timestamp_high = (micros >> 32) & 0x00FFFFFF;
    log_data.timestamp_low  = micros & 0xFFFFFFFF;
    switch (data_type){
        case odrive::command_ids::Get_Bus_Voltage_Current:
            log_data.data_1.float_data = target->get_vbus_voltage();
            log_data.data_2.float_data = target->get_vbus_current();
            break;
        case odrive::Get_Torques:
            log_data.data_1.float_data = target->get_torque_target();
            log_data.data_2.float_data = target->get_torque_estimate();
            break;
        case odrive::Get_Encoder_Estimates:
            log_data.data_1.float_data = target->get_pos_estimate();
            log_data.data_2.float_data = target->get_vel_estimate();
            break;
        case odrive::Get_Iq:
            log_data.data_1.float_data = target->get_Iq_setpoint();
            log_data.data_2.float_data = target->get_Iq_measured();
            break;
        default:
            break;
    }
    return log_data;
}

void HighSpeedLogger::add_log(HighSpeedLogger::LogData log_data){
    if (this->log_data_buffer.current_message >= 6){
        return;  // If we've run out of space, don't add any more data
    }
    LogDataMessage current_message = this->log_data_buffer.log_data_messages[this->log_data_buffer.current_message];
    current_message.log_data[current_message.total_data_points++] = log_data;
    if (current_message.total_data_points >= 56){
        this->log_data_buffer.current_message++;
    }

}

void HighSpeedLogger::log_callback(HighSpeedLogger *logger, odrive::command_ids command_id) {
    // Disable interrupts for the brief time it takes to copy the data
    // from the odrive to the log_data_buffer
    __disable_irq();
    if (logger->odrive == nullptr) return;
    LogData log_data = logger->get_log_data(logger->odrive, command_id);
    logger->add_log(log_data);
    __enable_irq();

}


