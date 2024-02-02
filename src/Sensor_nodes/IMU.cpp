//
// Created by Jay on 10/24/2023.
//

#include "IMU.h"
#include "Main_Helpers/BreadCrumbs.h"

//volatile uint32_t interrupt_count = 0;
//volatile uint32_t last_interrupt_time = 0;

void IMU::initialize(){
    DROP_CRUMB();
    this->imu_msg->header.frame_id = log_buffer;
    this->imu_msg->orientation.x = nanf("");
    this->imu_msg->orientation.y = nanf("");
    this->imu_msg->orientation.z = nanf("");
    this->imu_msg->orientation.w = nanf("");
    this->imu_msg->angular_velocity.x = nanf("");
    this->imu_msg->angular_velocity.y = nanf("");
    this->imu_msg->angular_velocity.z = nanf("");
    this->imu_msg->linear_acceleration.x = nanf("");
    this->imu_msg->linear_acceleration.y = nanf("");
    this->imu_msg->linear_acceleration.z = nanf("");

}

void IMU::update() {
    DROP_CRUMB();
    if (!this->sensor->is_valid()) return;
    if (this->sensor->get_last_update_time() < 100000) return;
    this->imu_msg->header.stamp = this->node_handle->now();
    this->imu_msg->orientation.x = this->imu_data.orientation.x;
    this->imu_msg->orientation.y = this->imu_data.orientation.y;
    this->imu_msg->orientation.z = this->imu_data.orientation.z;
    this->imu_msg->orientation.w = this->imu_data.orientation.w;
    this->imu_msg->angular_velocity.x = this->imu_data.angular_velocity.x;
    this->imu_msg->angular_velocity.y = this->imu_data.angular_velocity.y;
    this->imu_msg->angular_velocity.z = this->imu_data.angular_velocity.z;
    this->imu_msg->linear_acceleration.x = this->imu_data.linear_acceleration.x;
    this->imu_msg->linear_acceleration.y = this->imu_data.linear_acceleration.y;
    this->imu_msg->linear_acceleration.z = this->imu_data.linear_acceleration.z;
//    this->imu_msg->header.seq = this->imu_data.seq;
}

EStopDevice::TRIP_LEVEL IMU::tripped(char* tripped_device_name, char* tripped_device_message) {
    DROP_CRUMB();
    char temp[100];
    EStopDevice::TRIP_LEVEL tripped = NO_FAULT;
    sprintf(tripped_device_name, "IMU");
    sprintf(tripped_device_message, "");

    if (this->imu_data.flags) {
        sprintf(temp, "IMU FAULT 0x%02X-", this->imu_data.flags);
        strcat(tripped_device_message, temp);
        tripped = FAULT;
    }

    if (this->sensor->is_valid()) {
        if (this->sensor->get_last_update_time() > 100000) {
            sprintf(temp, "IMU DATA STALE-");
            strcat(tripped_device_message, temp);
            tripped = FAULT;
        }
    } else {
        sprintf(temp, "NO IMU DATA-");
        strcat(tripped_device_message, temp);
        tripped = FAULT;
    }

    // Remove the trailing dash if there is one
    if (tripped) tripped_device_message[strlen(tripped_device_message) - 1] = '\0';
    return tripped;
}