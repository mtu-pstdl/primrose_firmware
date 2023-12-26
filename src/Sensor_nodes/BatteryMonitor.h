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
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/sensor_msgs/BatteryState.h"

#include "Misc/EStopController.h"
#include "../../.pio/libdeps/teensy40/VeDirectFrameHandler/VeDirectFrameHandler.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <ADC.h>

#define VE_DIRECT_SERIAL Serial5

class BatteryMonitor : public ROSNode, public EStopDevice {


private:

    VeDirectFrameHandler* battery_interface;

    ros::NodeHandle* node_handle;

    sensor_msgs::BatteryState* battery_state_msg;

    ros::Subscriber<std_msgs::Int32MultiArray, BatteryMonitor> battery_sub;

    EStopController* estop_controller;

    uint8_t data_buffer[255] = {0};

    uint32_t last_update = 0;

    void battery_callback(const std_msgs::Int32MultiArray& msg){

    }

    void calculate_power_draw(float_t bus_voltage){

    }

public:

    char debug_string[1000] = {0};

    explicit BatteryMonitor(EStopController* estop_controller, sensor_msgs::BatteryState* battery_state_msg) :
            battery_sub("/mciu/battery_monitor", &BatteryMonitor::battery_callback, this) {

        this->battery_interface = new VeDirectFrameHandler();
        VE_DIRECT_SERIAL.begin(19200, SERIAL_8N1);
        VE_DIRECT_SERIAL.addMemoryForRead(data_buffer, 255);

        this->estop_controller = estop_controller;
        this->battery_state_msg = battery_state_msg;
        this->battery_state_msg->header.frame_id = "battery_link";
        this->battery_state_msg->voltage = NAN;
        this->battery_state_msg->current = NAN;
        this->battery_state_msg->charge = NAN;

        this->battery_state_msg->serial_number = "PRIMROSE_MCIU_BATTERY_MONITOR";
        this->battery_state_msg->location = "SOMEWHERE_ON_THE_MOON";

        // Setup battery state topic
        this->battery_state_msg->design_capacity = 0;
        this->battery_state_msg->power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
        this->battery_state_msg->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        this->battery_state_msg->power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        this->battery_state_msg->present = false;  // Until we get our first update from the BMS there is no battery
    }

    void update() override {
        uint32_t parse_start = micros();
        VE_DIRECT_SERIAL.write("V\r\n");
        sprintf(this->debug_string, "Battery Data: %d", VE_DIRECT_SERIAL.available());
        while (VE_DIRECT_SERIAL.available() && micros() - parse_start < 1000) {
            last_update = millis();
            this->battery_interface->rxData(VE_DIRECT_SERIAL.read());
        }

        for (int i = 0; i < this->battery_interface->veEnd; i++) {
            sprintf(this->debug_string, "%s\n%s = %s", this->debug_string, this->battery_interface->veName[i],
                    this->battery_interface->veValue[i]);
        }
    }

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(this->battery_sub);
        this->node_handle = node_handle;
    }

    EStopDevice::TRIP_LEVEL tripped(char* tripped_device_name, char* tripped_device_message) override {
        if (millis() - last_update > 1000) {
            sprintf(tripped_device_name, "Battery Monitor");
            sprintf(tripped_device_message, "No VE.Direct Data");
            return EStopController::TRIP_LEVEL::FAULT;
        }
        return EStopDevice::TRIP_LEVEL::NO_FAULT;
    }
};


#endif //PRIMROSE_MCIU_BATTERYMONITOR_H
