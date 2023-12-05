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

#include <utility>


class LoadCells : public ROSNode {

    std_msgs::Int32MultiArray* output_topic;

    uint8_t sensor_id;

    char* name = new char[30];

    ros::Subscriber<std_msgs::Int32MultiArray, LoadCells> command_sub;

    ADAU_Sensor* sensor;

    struct data_struct {
        int32_t sensor[4];  // Fixed point, x100
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
        command_sub.topic_ = this->name;
        sprintf(this->name, "/mciu/load_cells/%s/command", name);
        this->sensor = new ADAU_Sensor(sensor_id, &data, sizeof(data_struct));
    }

//    void update() override;

    void publish() override;

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(command_sub);
    }

    void control_callback(const std_msgs::Int32MultiArray& msg);

};


#endif //TEENSYCANTRANSCEIVER_LOADCELLS_H
