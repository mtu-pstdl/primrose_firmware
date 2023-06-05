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
#include "../../.pio/libdeps/teensy40/HX711/src/HX711.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include <EEPROM.h>

#include <utility>

#define EEPROM_CALIBRATION_ADDRESS_START 0xFF

class LoadCells : public ROSNode {

    int total_load_cells;

    char* subscriber_name;
    char** name_strings;
    char** value_strings;

    uint32_t eeprom_address;

    int32_t* data;
    int32_t total_weight = 0;
    bool tare_flag = false;

    HX711** load_cells;

    String name;

    diagnostic_msgs::DiagnosticStatus* diagnostic_topic;
    std_msgs::Int32MultiArray* output_topic;
    ros::Subscriber<std_msgs::Int32MultiArray, LoadCells> control_sub;

    bool* connected;

    void message_callback(const std_msgs::Int32MultiArray& msg);

    void tare();

    int32_t get_offset(int load_cell_number) const {
        int32_t offset = 0;
        EEPROM.get(this->eeprom_address + (load_cell_number * sizeof(int32_t)), offset);
    }

    void set_offset(int load_cell_number, int32_t offset) const {
        EEPROM.put(this->eeprom_address + (load_cell_number * sizeof(int32_t)), offset);
    }

    uint8_t online_load_cells() {
        uint8_t online = 0;
        for (int i = 0; i < this->total_load_cells; i++) {
            if (this->connected[i]) {
                online++;
            }
        }
        return online;
    }

    void update_diagnostics_topic(){
        this->diagnostic_topic->level = 0;
        this->diagnostic_topic->message = "All Ok";
        // check if any of the load cells are disconnected
        for (int i = 0; i < total_load_cells; i++) {
            if (!this->connected[i]) {
                this->diagnostic_topic->level = 1;
                this->diagnostic_topic->message = "Missing Load Cells";
                break;
            }
        }
        // Check if all load cells are disconnected
        for (int i = 0; i < total_load_cells; i++) {
            if (this->connected[i]) {
                break;
            }
            if (i == total_load_cells - 1) {
                this->diagnostic_topic->level = 2;
                this->diagnostic_topic->message = "No Load Cells";
            }
        }
    }

public:

    uint32_t get_used_eeprom() const {
        return this->eeprom_address + (this->total_load_cells * sizeof(int32_t)) + 1;
    }

    LoadCells(int total_load_cells, int* clock_pins, int* data_pins, float* calibration_factors,
              diagnostic_msgs::DiagnosticStatus* status, std_msgs::Int32MultiArray* output_topic,
                String disp_name, uint32_t eeprom_address) :
            control_sub("template_for_later", &LoadCells::message_callback, this) {

        this->load_cells = new HX711*[total_load_cells];
        this->total_load_cells = total_load_cells;
        this->name = disp_name;
        this->eeprom_address = eeprom_address;

        this->subscriber_name = new char[50];
        this->control_sub.topic_ = subscriber_name;
        sprintf(subscriber_name, "/mciu/%s/loadcells/control", disp_name.c_str());

        this->diagnostic_topic = status;
        this->output_topic = output_topic;
        this->output_topic->data_length = (total_load_cells + 1) * 2;
        this->output_topic->data = new int32_t[(total_load_cells + 1) * 2];
        this->output_topic->data[0] = 0;

        this->name_strings = new char*[(total_load_cells + 1) * 2];
        this->value_strings = new char*[(total_load_cells + 1) * 2];
        this->connected = new bool[total_load_cells];
        this->data = new int32_t[total_load_cells];

        this->diagnostic_topic->level = 0;
        this->diagnostic_topic->name = this->name.c_str();
        this->diagnostic_topic->hardware_id = "Load_Cells";
        this->diagnostic_topic->values_length = (total_load_cells + 1) * 2;
        this->diagnostic_topic->values = new diagnostic_msgs::KeyValue[(total_load_cells + 1) * 2];
        this->diagnostic_topic->values[0].key = "Total weight";

        for (int i = 0; i < (total_load_cells + 1) * 2; i++) {
            this->name_strings[i] = new char[30];
            this->value_strings[i] = new char[30];
            this->diagnostic_topic->values[i].key = this->name_strings[i];
            this->diagnostic_topic->values[i].value = this->value_strings[i];
        }


        // Assign each string to the correct key
        for (int i = 2; i < total_load_cells + 2; i++) {
            sprintf(this->name_strings[i], "Load cell %.2d weight", i - 1);
            sprintf(this->value_strings[i], "Connecting...");
        }

        for (int i = 2; i < total_load_cells + 2; i++) {
            sprintf(this->name_strings[i + total_load_cells], "Load cell %.2d offset", i - 1);
            sprintf(this->value_strings[i + total_load_cells], "Connecting...");
        }

        sprintf(this->name_strings[0], "Total weight");
        sprintf(this->value_strings[0], "Loading...");

        sprintf(this->name_strings[1], "Total offset");
        sprintf(this->value_strings[1], "Loading...");

        for (int i = 0; i < total_load_cells; i++) {
            this->load_cells[i] = new HX711();
            this->load_cells[i]->begin(data_pins[i], clock_pins[i]);
            if (this->load_cells[i]->wait_ready_timeout(250)){
                this->connected[i] = false;
            } else {
                this->connected[i] = true;
            }

            this->load_cells[i]->set_scale(calibration_factors[i]);
            // Get the offset from EEPROM
            this->load_cells[i]->set_offset(get_offset(i));
        }

        for (int i = total_load_cells + 2; i < (total_load_cells * 2) + 2; i++) {
            sprintf(value_strings[i], "%ld kg", get_offset(i - total_load_cells - 2));
        }

        // Calculate the total offset
        int32_t total_offset = 0;
        for (int i = 0; i < total_load_cells; i++) {
            total_offset += get_offset(i);
        }
        sprintf(value_strings[1], "%ld kg", total_offset);

        this->update_diagnostics_topic();
    }

    void update() override;

    void publish() override;

    void subscribe(ros::NodeHandle *node_handle) override;

};


#endif //TEENSYCANTRANSCEIVER_LOADCELLS_H
