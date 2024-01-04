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
#include "Main_Helpers/BreadCrumbs.h"

#include <utility>

/**
 * The LoadCell takes data from the ADAU load cell sensor and publishes it to the ROS network.
 * @note LoadCells are E-Stop trip devices and must be attached to an EStopController object.
 */
class LoadCells : public ROSNode, public EStopDevice {

    std_msgs::Int32MultiArray* output_topic;

    uint8_t sensor_id;

    union OutputArray {
        struct OutputData {
            int32_t sensor1;    // Fixed point, x100
            int32_t sensor2;
            int32_t sensor3;
            int32_t sensor4;
            uint8_t flags;      // First 4 bits are error flags for each sensor
        } data;
        int32_t raw_array[5];  // The raw array of data to be sent over the serial bus
    } output_data = {};

    char* topic_name = new char[30];
    char* name = new char[30];

    ros::Subscriber<std_msgs::Int32MultiArray, LoadCells> command_sub;

    ADAU_Sensor* sensor;

    #pragma pack(push, 1) // Remove all padding from the data structure to reduce transmission size
    struct data_struct {
        int32_t sensor[4];
        uint8_t flags;      // First 4 bits are error flags for each sensor
    };
    #pragma pack(pop) // End of data structure

    data_struct data = {
            .sensor = {0, 0, 0, 0},
            .flags = 0b00001111
    };

public:

    LoadCells(uint8_t sensor_id,  const char* name, std_msgs::Int32MultiArray* output_topic) :
            command_sub("template_for_later", &LoadCells::control_callback, this){
        this->sensor_id = sensor_id;
        this->output_topic = output_topic;
        this->output_topic->data_length = sizeof (output_data.raw_array) / sizeof (output_data.raw_array[0]);
        this->output_topic->data = data.sensor;
        // Change the name of the command topic to the correct name
        command_sub.topic_ = this->topic_name;
        sprintf(this->topic_name, "/mciu/%s/Load_cells/command", name);
        sprintf(this->name, "%s", name);
        this->sensor = new ADAU_Sensor(sensor_id, &data, sizeof(data_struct));
    }

    void update() override {

    }

    void subscribe(ros::NodeHandle *node_handle) override {
        node_handle->subscribe(command_sub);
    }

    void control_callback(const std_msgs::Int32MultiArray& msg);

    EStopDevice::TRIP_LEVEL tripped(char* tripped_device_name, char* tripped_device_message) override {
        DROP_CRUMB();
        sprintf(tripped_device_name, "Load Cells: %s", this->name);
        char temp[100]{};
        TRIP_LEVEL tripped = NO_FAULT;
        if (!sensor->is_valid()) {
            sprintf(temp, "ADAU DATA INVALID-");
            strlcat(tripped_device_message, temp, 100);
            tripped = FAULT;
        } else if (sensor->get_last_update_time() > 100000) {
            sprintf(temp, "ADAU DATA STALE-");
            strlcat(tripped_device_message, temp, 100);
            tripped = FAULT;
        }
        if (data.flags != 0) {
            sprintf(temp, "ERROR FLAGS: ");
            for (int i = 7; i >= 0; i--) {
                if (data.flags & (1 << i)) {
                    sprintf(temp, "%s%d ", temp, 1);
                } else {
                    sprintf(temp, "%s%d ", temp, 0);
                }
            }
            strlcat(tripped_device_message, temp, 100);
            tripped = FAULT;
        }
        if (tripped) tripped_device_message[strlen(tripped_device_message) - 1] = '\0';
        return tripped;
    }

};


#endif //TEENSYCANTRANSCEIVER_LOADCELLS_H
