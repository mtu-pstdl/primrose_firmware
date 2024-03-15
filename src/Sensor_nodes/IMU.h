//
// Created by Jay on 10/24/2023.
//

#ifndef PRIMROSE_MCIU_IMU_H
#define PRIMROSE_MCIU_IMU_H


#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/sensor_msgs/Imu.h"
//#include <SPI.h>
//#include "../../.pio/libdeps/teensy40/Adafruit BNO08x/src/Adafruit_BNO08x.h"
#include "Misc/EStopDevice.h"
#include "ADAU_Interfaces/ADAU_Sensor.h"

//Adafruit_BNO08x imu = Adafruit_BNO08x(-1);
// Declare global sensor message
//sh2_SensorValue_t sensor_value = {};

/**
 * The IMU class takes data from the BNO085 IMU sensor and publishes it to the ROS network.
 *
 * @note IMUs are E-Stop trip devices and must be attached to an EStopController object.
 */
class IMU: public ROSNode, public EStopDevice {

#pragma pack(push, 1)
    struct ImuData {
        float_t x = 0;
        float_t y = 0;
        float_t z = 0;
        float_t w = 0;

        uint8_t flags = 0x00;

        uint32_t seq = 0;
    } imu_data;
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
//        imu.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire2, 0);
//        imu.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
//        imu.enableReport(SH2_ACCELEROMETER, 10000);
//        imu.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
//        imu.enableReport(SH2_TEMPERATURE, 10000);
//        sensor_value.
    }

    void subscribe(ros::NodeHandle* nh) override {
        this->node_handle = nh;
    }

    void update() override;

    EStopDevice::TRIP_LEVEL tripped(char* tripped_device_name, char* tripped_device_message) override;

    void initialize();
};


#endif //PRIMROSE_MCIU_IMU_H
