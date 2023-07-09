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
#include "../../.pio/libdeps/teensy40/TLI4971-Current-Sensor/src/TLI4971.h"

#define BATTERY_MONITOR_EEPROM_ADDRESS 3500

#define BATTERY_MAX_CAPACITY  5120 // in W/h
#define BATTERY_NORM_CAPACITY 4600 // in W/h
#define BATTERY_MIN_CAPACITY  600 // in W/h

#define AREF_PIN A0
#define VREF_PIN A1

class BatteryMonitor : public ROSNode {


private:
    diagnostic_msgs::DiagnosticStatus* diagnostic_topic;

    TLI4971 CurrentSensor = TLI4971(AREF_PIN, VREF_PIN, 120, 5, 0, 0, 0, false);

    char*   battery_status;
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

    void load_data(){
        saved_data data;
        BatteryData slots[4];
        EEPROM.get(BATTERY_MONITOR_EEPROM_ADDRESS, data);
        for (int i = 0; i < 4; i++) {
            if (data.saved_battery_data[i].crc == calc_crc(data.saved_battery_data[i])) {
                this->battery_data = data.saved_battery_data[i];
                break;
            }
        }

    }

    ros::Subscriber<std_msgs::Int32MultiArray, BatteryMonitor> battery_sub;

    EStopController* estop_controller;

    enum BatteryCommands {
        SET_FUllY_CHARGED = 0,
        INCREASE_CHARGE = 1,
        DECREASE_CHARGE = 2,
    };

    void battery_callback(const std_msgs::Int32MultiArray& msg){
        switch (msg.data[0]){
            case SET_FUllY_CHARGED:
                this->battery_data.estimated_remaining_capacity = BATTERY_NORM_CAPACITY;
                break;
            case INCREASE_CHARGE:
                this->battery_data.estimated_remaining_capacity += (msg.data[1] * 0.01f) * BATTERY_MAX_CAPACITY;
                if (this->battery_data.estimated_remaining_capacity > BATTERY_MAX_CAPACITY)
                    this->battery_data.estimated_remaining_capacity = BATTERY_MAX_CAPACITY;
                break;
            case DECREASE_CHARGE:
                this->battery_data.estimated_remaining_capacity -= (msg.data[1] * 0.01f) * BATTERY_MAX_CAPACITY;
                if (this->battery_data.estimated_remaining_capacity < 0)
                    this->battery_data.estimated_remaining_capacity = 0;
                break;
        }
    }

    float_t calculate_remaining_capacity(){
        return 0;
    }

    float_t calculate_estimated_time_remaining(){
        return 0;
    }

//    void calculate_power_draw(){
//        this->bus_current = analogRead(CURRENT_SENSOR_PIN) * 0.0005;
//        this->battery_data.total_session_current_draw += this->bus_current;
//        this->battery_data.all_time_power_draw += this->bus_current;
//    }

public:

    explicit BatteryMonitor(diagnostic_msgs::DiagnosticStatus* status, EStopController* estop_controller) :
            battery_sub("/mciu/battery_monitor", &BatteryMonitor::battery_callback, this) {
        this->diagnostic_topic = status;
        this->estop_controller = estop_controller;

        this->battery_status = new char[25];
        sprintf(this->battery_status, "OK");

        this->diagnostic_topic->name = "Monitor";
        this->diagnostic_topic->message = this->battery_status;
        this->diagnostic_topic->hardware_id = "HVDC Bus";
        this->diagnostic_topic->values_length = 6;
        this->diagnostic_topic->values = new diagnostic_msgs::KeyValue[6];
        this->diagnostic_topic->values[0].key = "Main Bus Voltage";
        this->diagnostic_topic->values[1].key = "Main Bus Current";
        this->diagnostic_topic->values[2].key = "High Voltage Contactor";
        this->diagnostic_topic->values[3].key = "Estimated Remaining Capacity";
        this->diagnostic_topic->values[4].key = "Total Session Power Draw";
        this->diagnostic_topic->values[5].key = "All Time Power Draw";
        for (int i = 0; i < this->diagnostic_topic->values_length; i++){
            this->diagnostic_topic->values[i].value = new char[20];
            sprintf(this->diagnostic_topic->values[i].value, "");
        }
    }

    void update_bus_voltage(float_t voltage){
        this->bus_voltage = voltage;
    }

    void update_status(){
        if (!this->estop_controller->is_high_voltage_enabled()) {
            sprintf(this->battery_status, "HVDC Contactor Open");
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::ERROR;
        } else if (isnanf(this->bus_voltage)) {
                sprintf(this->battery_status, "No HVDC Bus Voltage");
                this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::ERROR;
        } else if (this->battery_data.estimated_remaining_capacity < BATTERY_MIN_CAPACITY) {
            sprintf(this->battery_status, "Low Estimated Capacity");
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::WARN;
        } else if (this->bus_voltage < 45) {
            sprintf(this->battery_status, "Low HVDC Bus Voltage (%05.2f V)", this->bus_voltage);
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::WARN;
        } else {
            sprintf(this->battery_status, "OK (%05.2f V)", this->bus_voltage);
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::OK;
        }
    };

    void update() override {
        this->update_status();
        sprintf(this->diagnostic_topic->values[0].value, "%05.2f V", this->bus_voltage);
        sprintf(this->diagnostic_topic->values[1].value, "%05.2f A", this->bus_current);
        sprintf(this->diagnostic_topic->values[2].value, "%s",
                this->estop_controller->is_high_voltage_enabled() ? "Closed*" : "Open");
        sprintf(this->diagnostic_topic->values[3].value, "%07.2f W/h (~%05.2f%%)",
                this->battery_data.estimated_remaining_capacity,
                (this->battery_data.estimated_remaining_capacity / BATTERY_NORM_CAPACITY) * 100);
        sprintf(this->diagnostic_topic->values[4].value, "%010.2f W/h", this->battery_data.total_session_power_draw);
        sprintf(this->diagnostic_topic->values[5].value, "%010.2f W/h", this->battery_data.all_time_power_draw);
    }

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(this->battery_sub);
    }
};


#endif //PRIMROSE_MCIU_BATTERYMONITOR_H
