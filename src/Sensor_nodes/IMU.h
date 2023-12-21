//
// Created by Jay on 10/24/2023.
//

#ifndef PRIMROSE_MCIU_IMU_H
#define PRIMROSE_MCIU_IMU_H


#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/sensor_msgs/Imu.h"
#include <SPI.h>
#include "../../.pio/libdeps/teensy40/Adafruit BNO08x/src/Adafruit_BNO08x.h"
#include "Misc/EStopDevice.h"

#define SPI_BUS SPI1
#define CS_PIN  32
#define IRQ_PIN 28
#define RST_PIN 29

class IMU: public ROSNode, public EStopDevice {

private:

    sh2_SensorValue_t sensor_value{};

    sensor_msgs::Imu* imu_msg;

    char log_buffer[256]{}; // For sending ros log messages
    bool config_success{};
    uint32_t last_report_time = 0;

    ros::NodeHandle* node_handle = nullptr;

    char failure_message[256]{};

    void enable_reporting();

public:

    explicit IMU(sensor_msgs::Imu* imu_msg){
        this->imu_msg = imu_msg;
        this->initialize();
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

    void initialize();
};


#endif //PRIMROSE_MCIU_IMU_H
