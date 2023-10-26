//
// Created by Jay on 10/24/2023.
//

#ifndef PRIMROSE_MCIU_IMU_H
#define PRIMROSE_MCIU_IMU_H


#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/sensor_msgs/Imu.h"
#include <SPI.h>
#include "../../.pio/libdeps/teensy40/SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library/src/ICM_20948.h"

#define SPI_BUS SPI
#define CS_PIN 10

class IMU: public ROSNode {

private:

    sensor_msgs::Imu* imu_msg;
    ICM_20948_SPI imu;

    ros::NodeHandle* node_handle;

public:

    IMU(sensor_msgs::Imu* imu_msg){
        this->imu_msg = imu_msg;
        this->imu_msg->header.frame_id = "imu_link_establishing";
        digitalWriteFast(CS_PIN, HIGH); // Deselect
        SPI_BUS.begin();
        this->imu.begin(CS_PIN, SPI_BUS);
        boolean config_success = true;
        config_success &= (this->imu.initializeDMP() == ICM_20948_Stat_Ok);
        config_success &= (this->imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
        config_success &= (this->imu.enableFIFO() == ICM_20948_Stat_Ok);
        // Enable the DMP
        config_success &= (this->imu.enableDMP() == ICM_20948_Stat_Ok);
        // Reset DMP
        config_success &= (this->imu.resetDMP() == ICM_20948_Stat_Ok);
        // Reset FIFO
        config_success &= (this->imu.resetFIFO() == ICM_20948_Stat_Ok);

        if (!config_success) {
            this->imu_msg->header.frame_id = "imu_link_failed";
        } else {
            this->imu_msg->header.frame_id = "imu_link_up";
        }
    }

    void subscribe(ros::NodeHandle* nh) override {
        this->node_handle = nh;
    }

    void update() override;

    void publish() override;

};


#endif //PRIMROSE_MCIU_IMU_H
