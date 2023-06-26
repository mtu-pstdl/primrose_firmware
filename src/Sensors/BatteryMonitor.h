//
// Created by Jay on 6/21/2023.
//

#ifndef PRIMROSE_MCIU_BATTERYMONITOR_H
#define PRIMROSE_MCIU_BATTERYMONITOR_H


#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/KeyValue.h"
#include <Arduino.h>
#include <EEPROM.h>

class BatteryMonitor : public ROSNode {


private:
    diagnostic_msgs::DiagnosticStatus* diagnostic_topic;

public:

    BatteryMonitor(diagnostic_msgs::DiagnosticStatus* status) {
        this->diagnostic_topic = status;

        this->diagnostic_topic->name = "Battery";
        this->diagnostic_topic->hardware_id = "DC Bus";
        this->diagnostic_topic->values_length = 4;
        this->diagnostic_topic->values = new diagnostic_msgs::KeyValue[4];
        this->diagnostic_topic->values[0].key = "Main Bus Voltage";
        this->diagnostic_topic->values[1].key = "Main Bus Current";
        this->diagnostic_topic->values[2].key = "Total Session Current Draw";
        this->diagnostic_topic->values[3].key = "Total";
    }

};


#endif //PRIMROSE_MCIU_BATTERYMONITOR_H
