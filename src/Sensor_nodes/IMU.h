//
// Created by Jay on 10/24/2023.
//

#ifndef PRIMROSE_MCIU_IMU_H
#define PRIMROSE_MCIU_IMU_H


#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/sensor_msgs/Imu.h"
//#include <SPI.h>
#include "Misc/EStopDevice.h"
#include "ADAU_Interfaces/ADAU_Sensor.h"

/**
 * The IMU class takes data from the BNO085 IMU sensor and publishes it to the ROS network.
 *
 * @note IMUs are E-Stop trip devices and must be attached to an EStopController object.
 */
class IMU: public ROSNode, public EStopDevice {

#pragma pack(push, 1)
    struct IMUData {
        struct {
            float_t x;
            float_t y;
            float_t z;
            float_t w;
        } orientation;
        float_t orientation_covariance[9];
        struct {
            float_t x;
            float_t y;
            float_t z;
        } angular_velocity;
        float_t angular_velocity_covariance[9];
        struct {
            float_t x;
            float_t y;
            float_t z;
        } linear_acceleration;
        float_t linear_acceleration_covariance[9];
        uint32_t seq;
        uint8_t  flags;
    } imu_data = {};
#pragma pack(pop)

    // The IMU sensor

private:

    // Method that gets called when the IRQ pin goes low
//    static void irq_handler();

    sensor_msgs::Imu* imu_msg;

    char log_buffer[256]{}; // For sending ros log messages
    ADAU_Sensor* sensor = nullptr;

    ros::NodeHandle* node_handle = nullptr;

//    char failure_message[256]{};

public:

    explicit IMU(sensor_msgs::Imu* imu_msg){
        this->imu_msg = imu_msg;
        this->initialize();
        this->sensor = new ADAU_Sensor(0x07, &this->imu_data, sizeof(this->imu_data));
    }

    void subscribe(ros::NodeHandle* nh) override {
        this->node_handle = nh;
//        if (this->config_success) {
//            this->node_handle->loginfo("IMU initialized successfully");
//        } else {
//            this->node_handle->logerror("IMU failed to initialize");
//        }
    }

    void update() override;

    EStopDevice::TRIP_LEVEL tripped(char* tripped_device_name, char* tripped_device_message) override;

    void initialize();
};


#endif //PRIMROSE_MCIU_IMU_H
