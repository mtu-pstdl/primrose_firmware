//
// Created by Jay on 5/31/2023.
//

#ifndef TEENSYCANTRANSCEIVER_IMU_H
#define TEENSYCANTRANSCEIVER_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/sensor_msgs/Imu.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"

class IMU {

    Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
    ros::Subscriber<std_msgs::Int32MultiArray, IMU> command_sub;

    diagnostic_msgs::DiagnosticStatus* state_topic;

    char** diagnostic_names;
    char** diagnostic_values;

    void message_callback(const std_msgs::Int32MultiArray& msg) {

    }

public:

    IMU(diagnostic_msgs::DiagnosticStatus* status, sensor_msgs::Imu* imu_topic) :
        command_sub("temp_name", &IMU::message_callback, this) {



        if (!this->accel.begin()) {
            status->level = 2;
            status->message = "No IMU";
        }

    }

};


#endif //TEENSYCANTRANSCEIVER_IMU_H
