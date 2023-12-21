//
// Created by Jay on 10/24/2023.
//

#include "IMU.h"

Adafruit_BNO08x imu(RST_PIN);

void IMU::initialize(){
    imu.hardwareReset();
    if (!imu.begin_SPI(CS_PIN, IRQ_PIN, &SPI_BUS)){
        this->imu_msg->header.frame_id = "imu_link_failed";
        return;
    } else {
        this->imu_msg->header.frame_id = "imu_link_up";
    }
    for (int n = 0; n < imu.prodIds.numEntries; n++) {
        sprintf(log_buffer, "%sProduct ID: %lu, Version: %d.%d.%d Build: %lu\n",
                log_buffer,
                imu.prodIds.entry[n].swPartNumber,
                imu.prodIds.entry[n].swVersionMajor,
                imu.prodIds.entry[n].swVersionMinor,
                imu.prodIds.entry[n].swVersionPatch,
                imu.prodIds.entry[n].swBuildNumber);
        this->last_report_time = millis();
    }
    if (imu.wasReset()) {
        sprintf(log_buffer, "%sSensor reset detected\n", log_buffer);
    }
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
    this->enable_reporting();
}

void IMU::enable_reporting() {
    sprintf(failure_message, "IMU initialization failed: ");
    this->config_success = true;
    // Enable desired reports
    if (!imu.enableReport(SH2_GAME_ROTATION_VECTOR, 20000)) { // 20ms
        this->config_success = false;
        sprintf(failure_message, "%sFailed to enable SH2_GAME_ROTATION_VECTOR ", failure_message);
    }
    if (!imu.enableReport(SH2_ACCELEROMETER)) { // 20ms
        this->config_success = false;
        sprintf(failure_message, "%sFailed to enable SH2_ACCELEROMETER ", failure_message);
    }
    if (!imu.enableReport(SH2_GYROSCOPE_CALIBRATED)) {  // 20ms
        this->config_success = false;
        sprintf(failure_message, "%sFailed to enable SH2_GYROSCOPE_CALIBRATED ", failure_message);
    }
    imu.enableReport(SH2_TEMPERATURE);  // 20ms
    imu.enableReport(SH2_HUMIDITY);     // 20ms
    if (this->config_success){
        sprintf(failure_message, "");
    }
}

void IMU::update() {
    if (!this->config_success) return;
    if (imu.wasReset()) {
        this->enable_reporting();
    }
    char temp[100]{};
    if (!imu.getSensorEvent(&sensor_value)) return;
        this->imu_msg->header.stamp = this->node_handle->now();
        last_report_time = millis();
        // Update the timestamp of the message
        switch (sensor_value.sensorId){
            case 0x00:  // Bad sensor id
                break;
            case SH2_GAME_ROTATION_VECTOR:
                this->imu_msg->orientation.x = sensor_value.un.gameRotationVector.i;
                this->imu_msg->orientation.y = sensor_value.un.gameRotationVector.j;
                this->imu_msg->orientation.z = sensor_value.un.gameRotationVector.k;
                this->imu_msg->orientation.w = sensor_value.un.gameRotationVector.real;
                break;
            case SH2_ACCELEROMETER:
                this->imu_msg->linear_acceleration.x = sensor_value.un.accelerometer.x;
                this->imu_msg->linear_acceleration.y = sensor_value.un.accelerometer.y;
                this->imu_msg->linear_acceleration.z = sensor_value.un.accelerometer.z;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                this->imu_msg->angular_velocity.x = sensor_value.un.gyroscope.x;
                this->imu_msg->angular_velocity.y = sensor_value.un.gyroscope.y;
                this->imu_msg->angular_velocity.z = sensor_value.un.gyroscope.z;
                break;
            case SH2_TEMPERATURE:
                this->imu_msg->orientation_covariance[0] = *(float*) &sensor_value.un.temperature;
                break;
            default:
                sprintf(temp, "Message data: 0x%x", sensor_value.sensorId);
        }
//    this->imu_msg->header.frame_id = temp;
}

void IMU::publish() {
}


boolean IMU::tripped(char* tripped_device_name, char* tripped_device_message) {
    char temp[100];
    bool tripped = false;
    sprintf(tripped_device_name, "IMU");
    if (!this->config_success) {
        strcat(tripped_device_message, this->failure_message);
        tripped = true;
    }
    if (this->last_report_time < millis() - 1000) {
        sprintf(temp, "Not Reporting-");
        strcat(tripped_device_message, temp);
        tripped = true;
    }
    // Remove the trailing dash if there is one
    if (tripped) tripped_device_message[strlen(tripped_device_message) - 1] = '\0';
    return tripped;
}
