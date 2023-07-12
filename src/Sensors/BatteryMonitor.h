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

#define BATTERY_MONITOR_EEPROM_ADDRESS 3200
#define BATTERY_SAVE_DATA_INTERVAL 30000 // in ms (30 seconds)

#define BATTERY_MAX_CAPACITY  5120 // in W/h
#define BATTERY_NORM_CAPACITY 4600 // in W/h
#define BATTERY_MIN_CAPACITY  600 // in W/h

#define CURRENT_SENSOR_SCALE_FACTOR 0.0005 // 0.5 mV/A

#define BATTERY_AVERAGE_BUFFER_SIZE 20

#define AREF_PIN A0
#define VREF_PIN A1

#define CURRENT_SENSOR_PIN 14

class BatteryMonitor : public ROSNode {


private:
    diagnostic_msgs::DiagnosticStatus* diagnostic_topic;

//    TLI4971 CurrentSensor = TLI4971(AREF_PIN, VREF_PIN, 120, 5, 0, 0, 0, false);

    char*   battery_status;
    float_t inst_bus_voltage = 0;
    float_t inst_bus_current = 0;
    float_t inst_bus_power   = 0;

    uint16_t average_buffer_index = 0;
    float_t  bus_power_average_buffer[BATTERY_AVERAGE_BUFFER_SIZE] = {0};
    float_t  bus_power_average = 0;

    bool     save_data_flag = false;  // When true, allow data to be saved to EEPROM
    uint32_t save_data_timer = 0;     // Timer for saving data to EEPROM

    struct BatteryData {
        uint16_t crc = 0;                           // CRC16 checksum of the data
        uint32_t sequence_number = 0;               // incremented every time the data is saved
        float_t  total_session_power_draw = 0;      // in W/h
        float_t  estimated_remaining_capacity = 0;  // in W/h
        float_t  all_time_power_draw = 0;           // in W/h
    } battery_data; // stores power consumption data in EEPROM and RAM

    struct saved_data {
        BatteryData saved_battery_data[4];
    };  // Contains all power data save slots

    saved_data  eeprom_battery_data;   // stores power consumption data in EEPROM

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
        uint32_t max_sequence_number = 0;
        EEPROM.get(BATTERY_MONITOR_EEPROM_ADDRESS, eeprom_battery_data);
        for (auto & i : eeprom_battery_data.saved_battery_data) {
            if (i.crc == calc_crc(i)) {
                if (i.sequence_number > max_sequence_number) {
                    max_sequence_number = i.sequence_number;
                    this->battery_data = i;
                }
            }
        }
    }

    void save_data(bool force_save = false) {
        if (!save_data_flag && !force_save) return;
        if (millis() - save_data_timer < BATTERY_SAVE_DATA_INTERVAL && !force_save) return;
        // Save to the next slot in EEPROM
        this->battery_data.sequence_number++;
        this->battery_data.crc = calc_crc(this->battery_data);
        // Determine which slot has the lowest sequence number to overwrite
        uint8_t lowest_sequence_number_slot = 0;
        uint32_t lowest_sequence_number = UINT32_MAX;
        for (uint8_t i = 0; i < 4; i++) {
            if (eeprom_battery_data.saved_battery_data[i].sequence_number < lowest_sequence_number) {
                lowest_sequence_number = eeprom_battery_data.saved_battery_data[i].sequence_number;
                lowest_sequence_number_slot = i;
            }
        }
        // Write directly to that slots memory address and don't overwrite the other slots
        EEPROM.put(BATTERY_MONITOR_EEPROM_ADDRESS + (lowest_sequence_number_slot * sizeof(BatteryData)),
                   this->battery_data);
        save_data_timer = millis();
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
                this->save_data(true);
                break;
            case INCREASE_CHARGE:
                this->battery_data.estimated_remaining_capacity += (msg.data[1] * 0.01f) * BATTERY_MAX_CAPACITY;
                if (this->battery_data.estimated_remaining_capacity > BATTERY_MAX_CAPACITY)
                    this->battery_data.estimated_remaining_capacity = BATTERY_MAX_CAPACITY;
                this->save_data(true);
                break;
            case DECREASE_CHARGE:
                this->battery_data.estimated_remaining_capacity -= (msg.data[1] * 0.01f) * BATTERY_MAX_CAPACITY;
                if (this->battery_data.estimated_remaining_capacity < 0)
                    this->battery_data.estimated_remaining_capacity = 0;
                this->save_data(true);
                break;
        }
    }

    /**
     * @brief Calculate the estimated time remaining based on the average power draw over the last second
     * @return Estimated remaining operating time in hours or inf if the average power draw is 0
     */
    float_t calculate_estimated_time_remaining() const{
        if (this->bus_power_average == 0) return INFINITY;
        return this->battery_data.estimated_remaining_capacity / this->bus_power_average;
    }

    /**
     * @brief Write the estimated time remaining to a character buffer
     * @details Format DD HH:MM:SS unless the estimated time remaining is inf then write "inf"
     * @param buffer Character buffer to write to
     * @return void
     */
     void write_estimated_time_remaining(char* buffer) const {
        float_t estimated_time_remaining = this->calculate_estimated_time_remaining();
        if (estimated_time_remaining == INFINITY) {
            sprintf(buffer, "inf");
        } else {
            auto days    = (uint8_t) (estimated_time_remaining / 24);
            auto hours   = (uint8_t) (estimated_time_remaining - (days * 24));
            auto minutes = (uint8_t) ((estimated_time_remaining - (days * 24) - hours) * 60);
            auto seconds = (uint8_t) ((((estimated_time_remaining - (days * 24) - hours) * 60) - minutes) * 60);
            sprintf(buffer, "%02d %02d:%02d:%02d", days, hours, minutes, seconds);  // Format DD HH:MM:SS
        }
    }

    void calculate_power_draw(){
        this->inst_bus_current = analogRead(CURRENT_SENSOR_PIN) * CURRENT_SENSOR_SCALE_FACTOR;
//        if (!isnanf(this->inst_bus_voltage)) {
//            // Calculate power draw
//            this->inst_bus_power = this->inst_bus_voltage * this->inst_bus_current;
//            // Calculate total session power draw
//            this->battery_data.total_session_power_draw += this->inst_bus_voltage;
//            this->battery_data.all_time_power_draw += this->inst_bus_current;
//            this->battery_data.estimated_remaining_capacity -= this->inst_bus_power;
//            // Add to the average power draw
//            bus_power_average_buffer[this->average_buffer_index] = this->inst_bus_power;
//            this->average_buffer_index++;
//            if (this->average_buffer_index >= BATTERY_AVERAGE_BUFFER_SIZE) {  // Calculate average power draw
//                this->average_buffer_index = 0;
//                float_t sum = 0;
//                for (auto &i: bus_power_average_buffer) {
//                    sum += i;
//                }
//                this->bus_power_average = sum / BATTERY_AVERAGE_BUFFER_SIZE;
//            }
//        } else {
//            this->inst_bus_current = 0;
//            this->inst_bus_power = 0;
//            this->bus_power_average = 0;
//        }
    }

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
        this->diagnostic_topic->values_length = 7;
        this->diagnostic_topic->values = new diagnostic_msgs::KeyValue[7];
        this->diagnostic_topic->values[0].key = "Main Bus Voltage";
        this->diagnostic_topic->values[1].key = "Main Bus Current";
        this->diagnostic_topic->values[2].key = "High Voltage Contactor";
        this->diagnostic_topic->values[3].key = "Estimated Remaining Capacity";
        this->diagnostic_topic->values[4].key = "Estimated Time Remaining";
        this->diagnostic_topic->values[5].key = "Total Session Power Draw";
        this->diagnostic_topic->values[6].key = "All Time Power Draw";
        for (int i = 0; i < this->diagnostic_topic->values_length; i++){
            this->diagnostic_topic->values[i].value = new char[20];
            sprintf(this->diagnostic_topic->values[i].value, "");
        }
        load_data();
        pinMode(CURRENT_SENSOR_PIN, INPUT);
    }

    void update_bus_voltage(float_t voltage){
        this->inst_bus_voltage = voltage;
        this->calculate_power_draw();
    }

    void update_status(){
        if (!this->estop_controller->is_high_voltage_enabled()) {
            sprintf(this->battery_status, "HVDC Contactor Open");   // Indicates the HVDC contactor is open
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::ERROR;
        } else if (isnanf(this->inst_bus_voltage)) {
            sprintf(this->battery_status, "No HVDC Voltage Data");  // Indicates a problem with the CAN bus
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::ERROR;
        } else if (this->inst_bus_voltage > 56) {  // HVDC voltage above 56V
            sprintf(this->battery_status, "High HVDC Voltage (%05.2fV)", this->inst_bus_voltage);
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::ERROR;
        } else if (this->inst_bus_voltage < 44) {  // HVDC voltage below 44V
            sprintf(this->battery_status, "Low HVDC Voltage (%05.2fV)", this->inst_bus_voltage);
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::ERROR;
        } else if (this->calculate_estimated_time_remaining() < 0.75) {
            sprintf(this->battery_status, "Low ETR (%05.2fH)", this->calculate_estimated_time_remaining());
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::WARN;
        } else if (this->battery_data.estimated_remaining_capacity < BATTERY_MIN_CAPACITY) {
            sprintf(this->battery_status, "Low Estimated Capacity");
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::WARN;
        } else {
            sprintf(this->battery_status, "OK (%05.2fV | %05.2fA | %07.2fW/h)",
                    this->inst_bus_voltage, this->inst_bus_current, this->battery_data.estimated_remaining_capacity);
            this->diagnostic_topic->level = diagnostic_msgs::DiagnosticStatus::OK;
        }
    };

    void update() override {
        this->update_status();
        if (this->estop_controller->is_high_voltage_enabled()) {
            sprintf(this->diagnostic_topic->values[0].value, "%05.2f V", this->inst_bus_voltage);
        } else sprintf(this->diagnostic_topic->values[0].value, "Contactor Open");
        sprintf(this->diagnostic_topic->values[1].value, "%010.6f A", this->inst_bus_current);
        sprintf(this->diagnostic_topic->values[2].value, "%s",
                this->estop_controller->is_high_voltage_enabled() ? "Closed*" : "Open");
        sprintf(this->diagnostic_topic->values[3].value, "%07.2f W/h (~%05.2f%%)",
                this->battery_data.estimated_remaining_capacity,
                (this->battery_data.estimated_remaining_capacity / BATTERY_NORM_CAPACITY) * 100);
        this->write_estimated_time_remaining(this->diagnostic_topic->values[4].value);
        sprintf(this->diagnostic_topic->values[5].value, "%010.2f W/h", this->battery_data.total_session_power_draw);
        sprintf(this->diagnostic_topic->values[6].value, "%010.2f W/h", this->battery_data.all_time_power_draw);
        this->save_data_flag = this->estop_controller->is_high_voltage_enabled();
    }

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(this->battery_sub);
    }
};


#endif //PRIMROSE_MCIU_BATTERYMONITOR_H
