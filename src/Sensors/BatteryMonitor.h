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
#include "Misc/EStopController.h"
#include <Arduino.h>
#include <EEPROM.h>

#define BATTERY_MONITOR_EEPROM_ADDRESS 3500
#define CURRENT_SENSOR_PIN A0

class BatteryMonitor : public ROSNode {


private:
    diagnostic_msgs::DiagnosticStatus* diagnostic_topic;

    float_t bus_voltage = 0;
    float_t bus_current = 0;

    struct BatteryData {
        uint16_t crc = 0;                           // CRC16 checksum of the data
        uint32_t sequence_number = 0;               // incremented every time the data is saved
        float_t  total_session_power_draw = 0;      // in W/h
        float_t  all_time_power_draw = 0;           // in W/h
        float_t  estimated_remaining_capacity = 0;  // in W/h
    } battery_data; // stores power consumption data in EEPROM and RAM

    struct saved_data {
        BatteryData saved_battery_data[4];
    };  // Contains all power data save slots

    static uint16_t calc_crc(BatteryData odometer) {
        // Calculates CRC16 checksum of odometer data
        uint16_t crc = 0xFFFF;
        auto* odometer_bytes = (uint8_t*) &odometer;
        // Skip the first 4 bytes (changed) because it is not saved in EEPROM
        for (int i = 4; i < sizeof(BatteryData); i++) {
            crc ^= odometer_bytes[i];

            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc >>= 1;
                    crc ^= 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

//    void load_data(){
//        saved_data data;
//        EEPROM.get(BATTERY_MONITOR_EEPROM_ADDRESS, data);
//
//    }

    ros::Subscriber<std_msgs::Int32MultiArray, BatteryMonitor> battery_sub;

    EStopController* estop_controller;

    void battery_callback(const std_msgs::Int32MultiArray& msg){

    }

//    void calculate_power_draw(){
//        this->bus_current = analogRead(CURRENT_SENSOR_PIN) * 0.0005;
//        this->battery_data.total_session_current_draw += this->bus_current;
//        this->battery_data.all_time_power_draw += this->bus_current;
//    }

public:

    explicit BatteryMonitor(diagnostic_msgs::DiagnosticStatus* status, EStopController* estop_controller) :
            battery_sub("battery_monitor", &BatteryMonitor::battery_callback, this) {
        this->diagnostic_topic = status;
        this->estop_controller = estop_controller;

        pinMode(CURRENT_SENSOR_PIN, INPUT);

        this->diagnostic_topic->name = "Battery";
        this->diagnostic_topic->message = "All Ok";
        this->diagnostic_topic->hardware_id = "DC Bus";
        this->diagnostic_topic->values_length = 5;
        this->diagnostic_topic->values = new diagnostic_msgs::KeyValue[6];
        this->diagnostic_topic->values[0].key = "Main Bus Voltage";
        this->diagnostic_topic->values[1].key = "Main Bus Current";
        this->diagnostic_topic->values[2].key = "High Voltage Contactor";
        this->diagnostic_topic->values[3].key = "Estimated Remaining Capacity";
        this->diagnostic_topic->values[4].key = "Total Session Power Draw";
        this->diagnostic_topic->values[5].key = "All Time Power Draw";
        for (int i = 0; i < this->diagnostic_topic->values_length; i++){
            this->diagnostic_topic->values[i].value = new char[10];
            sprintf(this->diagnostic_topic->values[i].value, "");
        }
    }

    void update_bus_voltage(float_t voltage){
        this->bus_voltage = voltage;
    }

    void update() override {
        sprintf(this->diagnostic_topic->values[0].value, "%05.2f V", this->bus_voltage);
        sprintf(this->diagnostic_topic->values[1].value, "%05.2f A", this->bus_current);
        sprintf(this->diagnostic_topic->values[2].value, "%s",
                EStopController::is_high_voltage_enabled() ? "Closed*" : "Open");
        sprintf(this->diagnostic_topic->values[3].value, "%05.2f W/h", this->battery_data.estimated_remaining_capacity);
        sprintf(this->diagnostic_topic->values[4].value, "%05.2f W/h", this->battery_data.total_session_power_draw);
        sprintf(this->diagnostic_topic->values[5].value, "%05.2f W/h", this->battery_data.all_time_power_draw);
    }
};


#endif //PRIMROSE_MCIU_BATTERYMONITOR_H
