//
// Created by Jay on 6/21/2023.
//

#ifndef PRIMROSE_MCIU_BATTERYMONITOR_H
#define PRIMROSE_MCIU_BATTERYMONITOR_H


#include "ROSNode.h"
#include "VEDirect.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/KeyValue.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/sensor_msgs/BatteryState.h"

#include "Misc/EStopController.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <ADC.h>


#define BATTERY_MONITOR_EEPROM_ADDRESS 3200
#define BATTERY_SAVE_DATA_INTERVAL 30000 // in ms (30 seconds)

#define BATTERY_MAX_CAPACITY  5120 // in W/h
#define BATTERY_NORM_CAPACITY 5120 // in W/h
#define BATTERY_MIN_CAPACITY  850 // in W/h

// The scale factor is 10mv/A
#define CURRENT_SENSOR_SCALE_FACTOR 0.01 // in V/A (10 mV/A)
#define CURRENT_SENSOR_CAL_OFFSET 1.6331 // What voltage is zero

#define BATTERY_AVERAGE_BUFFER_SIZE 20

#define AREF_PIN A0
#define VREF_PIN A1

#define CURRENT_SENSOR_PIN 14

class BatteryMonitor : public ROSNode {


private:

    ros::NodeHandle* node_handle;

    sensor_msgs::BatteryState* battery_state_msg;

//    TLI4971 CurrentSensor = TLI4971(AREF_PIN, VREF_PIN, 120, 5, 0, 0, 0, false);

    float_t inst_bus_voltage = 0;
    float_t inst_bus_current = 0;
    float_t inst_bus_power   = 0;

    uint32_t last_update_time = 0;

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

    float_t analog_read_to_voltage(uint8_t pin, uint8_t resolution = 12) {
        return (float_t) analogRead(pin) * (3.3 / (float_t) pow(2, resolution));
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
                this->battery_data.total_session_power_draw = 0;
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

    void calculate_power_draw(float_t bus_voltage){
        this->inst_bus_current =
                (analog_read_to_voltage(CURRENT_SENSOR_PIN) - CURRENT_SENSOR_CAL_OFFSET)
                / CURRENT_SENSOR_SCALE_FACTOR * -1;
//        if (this->inst_bus_current < 0) this->inst_bus_current = 0;
        if(isnanf(bus_voltage)) bus_voltage = 51.2f;
        if(bus_voltage < 6) bus_voltage = 51.2f;
        // Calculate power draw
        this->inst_bus_power = bus_voltage * this->inst_bus_current;
        // Convert bus power to watt/hours so we can add it to the total power draw
        // Use the time since the last loop to calculate the watt/hours so that the reading is time based
        float_t consumption_rate = this->inst_bus_power * ((millis() - this->last_update_time) / 3600000.0f);
        this->last_update_time = millis();
        this->battery_data.total_session_power_draw += consumption_rate;
        this->battery_data.all_time_power_draw += consumption_rate;
        this->battery_data.estimated_remaining_capacity -= consumption_rate;
        // Add to the average power draw
        bus_power_average_buffer[this->average_buffer_index] = this->inst_bus_power;
        this->average_buffer_index++;
        if (this->average_buffer_index >= BATTERY_AVERAGE_BUFFER_SIZE) {  // Calculate average power draw
            this->average_buffer_index = 0;
            float_t sum = 0;
            for (auto &i: bus_power_average_buffer) {
                sum += i;
            }
            this->bus_power_average = sum / BATTERY_AVERAGE_BUFFER_SIZE;
        }
    }

public:

    explicit BatteryMonitor(EStopController* estop_controller, sensor_msgs::BatteryState* battery_state_msg) :
            battery_sub("/mciu/battery_monitor", &BatteryMonitor::battery_callback, this) {

        this->estop_controller = estop_controller;
        this->battery_state_msg = battery_state_msg;
        this->battery_state_msg->header.frame_id = "battery_link";
        this->battery_state_msg->voltage = NAN;
        this->battery_state_msg->current = NAN;
        this->battery_state_msg->charge = NAN;

        this->battery_state_msg->serial_number = "PRIMROSE_MCIU_BATTERY_MONITOR";
        this->battery_state_msg->location = "SOMEWHERE_ON_THE_MOON";

        // Setup battery state topic
        this->battery_state_msg->design_capacity = BATTERY_NORM_CAPACITY;
        this->battery_state_msg->power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
        this->battery_state_msg->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        this->battery_state_msg->power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        this->battery_state_msg->present = false;  // Until we get our first update from the BMS there is no battery



        load_data();

    }

    void reset_data(){
        this->battery_data.estimated_remaining_capacity = BATTERY_NORM_CAPACITY;
        this->battery_data.total_session_power_draw = 0;
        this->battery_data.all_time_power_draw = 0;
        this->save_data(true);
     }

    void update_bus_voltage(float_t voltage){
        this->inst_bus_voltage = voltage;
        this->calculate_power_draw(voltage);
    }

    void update_status(){

    };

    void update() override {
        this->save_data_flag = this->estop_controller->is_high_voltage_enabled();
        this->battery_state_msg->header.stamp = this->node_handle->now();
        this->save_data();
    }

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(this->battery_sub);
        this->node_handle = node_handle;
    }
};


#endif //PRIMROSE_MCIU_BATTERYMONITOR_H
