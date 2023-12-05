//
// Created by Jay on 10/24/2023.
//

#include "IMU.h"

void IMU::update() {
    // Read the FIFO
    icm_20948_DMP_data_t data;
    this->imu_msg->header.stamp = this->node_handle->now();
    if (!this->config_success) return;
    if (!this->imu.isConnected()) {
        this->imu_msg->header.frame_id = "imu_link_failed";
        return;
    } else {
        this->imu_msg->header.frame_id = "imu_link_up";
    }

    this->imu.readDMPdataFromFIFO(&data);
    // Update the message

    double q1 = ((double)data.PQuat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
    double q2 = ((double)data.PQuat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
    double q3 = ((double)data.PQuat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

    this->imu_msg->orientation.x = q1;
    this->imu_msg->orientation.y = q2;
    this->imu_msg->orientation.z = q3;
    this->imu_msg->orientation.w = q0;

    this->imu_msg->orientation_covariance[0] = 0;
    this->imu_msg->angular_velocity.x = NAN;
    this->imu_msg->angular_velocity.y = NAN;
    this->imu_msg->angular_velocity.z = NAN;
    this->imu_msg->angular_velocity_covariance[0] = 0;
    this->imu_msg->linear_acceleration.x = data.Raw_Accel.Data.X;
    this->imu_msg->linear_acceleration.y = data.Raw_Accel.Data.Y;
    this->imu_msg->linear_acceleration.z = data.Raw_Accel.Data.Z;
    this->imu_msg->linear_acceleration_covariance[0] = 0;

}

void IMU::publish() {
}


boolean IMU::tripped(char* tripped_device_name, char* tripped_device_message) {
    char temp[100];
    bool tripped = false;
    sprintf(tripped_device_name, "IMU");
    if (!this->config_success) {
        sprintf(temp, "INIT FAILED-");
        strcat(tripped_device_message, temp);
        tripped = true;
    }
    if (!this->imu.isConnected()) {
        sprintf(temp, "CONN LOST-");
        strcat(tripped_device_message, temp);
        tripped = true;
    }
    // Remove the trailing dash if there is one
    if (tripped) tripped_device_message[strlen(tripped_device_message) - 1] = '\0';
    return tripped;
}
