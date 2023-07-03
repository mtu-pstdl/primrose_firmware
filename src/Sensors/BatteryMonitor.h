//
// Created by Jay on 6/21/2023.
//

#ifndef PRIMROSE_MCIU_BATTERYMONITOR_H
#define PRIMROSE_MCIU_BATTERYMONITOR_H


#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/KeyValue.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"
#include <Arduino.h>
#include <EEPROM.h>

#define BATTERY_MONITOR_EEPROM_ADDRESS 0

class BatteryMonitor : public ROSNode {


private:
    diagnostic_msgs::DiagnosticStatus* diagnostic_topic;

    float_t bus_voltage_average = 0;

    ros::Subscriber<std_msgs::Int32MultiArray, BatteryMonitor> battery_sub;

    void battery_callback(const std_msgs::Int32MultiArray& msg){

    }

public:

    BatteryMonitor(diagnostic_msgs::DiagnosticStatus* status) :
            battery_sub("battery_monitor", &BatteryMonitor::battery_callback, this) {
        this->diagnostic_topic = status;

        this->diagnostic_topic->name = "Battery";
        this->diagnostic_topic->hardware_id = "DC Bus";
        this->diagnostic_topic->values_length = 4;
        this->diagnostic_topic->values = new diagnostic_msgs::KeyValue[4];
        this->diagnostic_topic->values[0].key = "Main Bus Voltage";
        this->diagnostic_topic->values[1].key = "Main Bus Current";
        this->diagnostic_topic->values[2].key = "Estimated Remaining Capacity";
        this->diagnostic_topic->values[2].key = "Total Session Current Draw";
        this->diagnostic_topic->values[3].key = "All Time Power Draw";
    }

    void update_bus_voltage(float_t voltage){

    }

};


#endif //PRIMROSE_MCIU_BATTERYMONITOR_H
