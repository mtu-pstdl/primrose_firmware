//
// Created by Jay on 5/26/2023.
//

#ifndef TEENSYCANTRANSCEIVER_LOADCELLS_H
#define TEENSYCANTRANSCEIVER_LOADCELLS_H

#include <Arduino.h>
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/UInt32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/KeyValue.h"
#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "ADAU_Interfaces/ADAU_Sensor.h"
#include "Misc/EStopDevice.h"

#include <utility>


class LoadCells : public ROSNode, public EStopDevice {

    std_msgs::Int32MultiArray* output_topic;

    uint8_t sensor_id;

    char* topic_name = new char[30];
    char* name = new char[30];

    ros::Subscriber<std_msgs::Int32MultiArray, LoadCells> command_sub;

    ADAU_Sensor* sensor;

    struct data_struct {
        int32_t sensor[4];
        uint8_t flags;      // First 4 bits are error flags for each sensor
    };

    data_struct data = {
            .sensor = {0, 0, 0, 0},
            .flags = 0b00001111
    };

public:

    LoadCells(uint8_t sensor_id,  const char* name, std_msgs::Int32MultiArray* output_topic) :
            command_sub("template_for_later", &LoadCells::control_callback, this){
        this->sensor_id = sensor_id;
        this->output_topic = output_topic;
        this->output_topic->data_length = 4;
        this->output_topic->data = data.sensor;
        // Change the name of the command topic to the correct name
        command_sub.topic_ = this->topic_name;
        sprintf(this->topic_name, "/mciu/%s/Load_cells/command", name);
        sprintf(this->name, "%s", name);
        this->sensor = new ADAU_Sensor(sensor_id, &data, sizeof(data_struct));
    }

    void update() override {

    }

    void publish() override;

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(command_sub);
    }

    void control_callback(const std_msgs::Int32MultiArray& msg);

    boolean tripped(char* tripped_device_name, char* tripped_device_message) override {
        if (data.flags != 0) {
            sprintf(tripped_device_name, "Load Cells: %s", this->name);
            sprintf(tripped_device_message, "ERROR FLAGS: ");
            for (int i = 0; i < 8; i++) {
                if (data.flags & (1 << i)) {
                    sprintf(tripped_device_message, "%s%d ", tripped_device_message, 1);
                } else {
                    sprintf(tripped_device_message, "%s%d ", tripped_device_message, 0);
                }
            }
            return true;
        }
        return false;
    }

};


#endif //TEENSYCANTRANSCEIVER_LOADCELLS_H
