//
// Created by Jay on 10/24/2023.
//

#ifndef PRIMROSE_MCIU_IMU_H
#define PRIMROSE_MCIU_IMU_H


#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/sensor_msgs/Imu.h"
#include <SPI.h>
#include "../../.pio/libdeps/teensy40/SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library/src/ICM_20948.h"
#include "Misc/EStopDevice.h"

#define SPI_BUS SPI1
#define CS_PIN  32
#define IRQ_PIN 29
#define RST_PIN 28

#ifndef ICM_20948_USE_DMP // If DMP is not included trigger a compile error
#error "DMP was not enabled in the IMU library please define ICM_20948_USE_DMP in ICM_20948_C.h (Line 29)"
#endif

// Initialization step bit masks
#define INIT_DMP          0x0001
#define ENABLE_DMP_SENSOR 0x0002
#define ENABLE_FIFO       0x0004
#define ENABLE_DMP        0x0008
#define RESET_DMP         0x0010
#define RESET_FIFO        0x0020

class IMU: public ROSNode, public EStopDevice {

private:

    sensor_msgs::Imu* imu_msg;
    ICM_20948_SPI imu;
    char log_buffer[100]; // For sending ros log messages
    bool config_success;

    ros::NodeHandle* node_handle = nullptr;

    char failure_message[100];

    void init_fail_formatter(uint16_t steps) {
        // List which steps failed
        failure_message[0] = '\0';
        strcat(failure_message, "INIT_FAILED: ");
        if (steps & INIT_DMP) strcat(failure_message, "INIT_DMP ");
        if (steps & ENABLE_DMP_SENSOR) strcat(failure_message, "ENABLE_DMP_SENSOR ");
        if (steps & ENABLE_FIFO) strcat(failure_message, "ENABLE_FIFO ");
        if (steps & ENABLE_DMP) strcat(failure_message, "ENABLE_DMP ");
        if (steps & RESET_DMP) strcat(failure_message, "RESET_DMP ");
        if (steps & RESET_FIFO) strcat(failure_message, "RESET_FIFO ");
    }

public:

    explicit IMU(sensor_msgs::Imu* imu_msg){
        this->imu_msg = imu_msg;
        digitalWriteFast(CS_PIN, HIGH); // Deselect
        SPI_BUS.begin();
//        digitalWrite(RST_PIN, HIGH);
        delay(10);
//        digitalWrite(RST_PIN, HIGH);
        pinMode(IRQ_PIN, INPUT);
        this->imu.begin(CS_PIN, SPI_BUS);
        // Check if the IMU is connected
//        if (!this->imu.isConnected()) {
//            this->config_success = false;
//            this->imu_msg->header.frame_id = "imu_link_down";
//            return;
//        }
        uint16_t steps = 0;
        steps |= (this->imu.initializeDMP() != ICM_20948_Stat_Ok) << 0;
        steps |= (this->imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) != ICM_20948_Stat_Ok) << 1;
        steps |= (this->imu.enableFIFO() != ICM_20948_Stat_Ok) << 2;
        // Enable the DMP (Digital Motion Processor)
        steps |= (this->imu.enableDMP() != ICM_20948_Stat_Ok) << 3;
        // Reset DMP
        steps |= (this->imu.resetDMP() != ICM_20948_Stat_Ok) << 4;
        // Reset FIFO
        steps |= (this->imu.resetFIFO() != ICM_20948_Stat_Ok) << 5;

        if (steps != 0) {
            this->init_fail_formatter(steps);
            this->imu_msg->header.frame_id = failure_message;
        } else {
            this->imu_msg->header.frame_id = "imu_link_ready";
        }
    }

    void subscribe(ros::NodeHandle* nh) override {
        this->node_handle = nh;
        if (this->config_success) {
            this->node_handle->loginfo("IMU initialized successfully");
        } else {
            this->node_handle->logerror("IMU failed to initialize");
        }
    }

    void update() override;

    void publish() override;

    boolean tripped(char* tripped_device_name, char* tripped_device_message) override;

};


#endif //PRIMROSE_MCIU_IMU_H
