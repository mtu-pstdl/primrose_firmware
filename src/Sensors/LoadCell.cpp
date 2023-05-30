//
// Created by Jay on 5/26/2023.
//

#include "LoadCells.h"


void LoadCells::publish() {

    for (int i = 0; i < total_load_cells; i++) {
        output_topic->data[i + 1] = data[i];
    }

    if (online_load_cells() == 0){
        sprintf(value_strings[0], "Unavailable");
    } else sprintf(value_strings[0], "%ld", this->total_weight);

    for (int i = 0; i < total_load_cells; i++) {
        if (connected[i]) {
            sprintf(value_strings[i + 1], "%ld", data[i]);
        } else {
            sprintf(value_strings[i + 1], "Unavailable");
        }
    }

}

void LoadCells::update() {
    // Read all connected load cells
    for (int i = 0; i < total_load_cells; i++) {
        if (connected[i]) {
            this->data[i] = load_cells[i]->read();
        } else {
            this->data[i] = INT32_MIN;
        }
    }
    // If more than 1 load cell is connected, calculate the total weight by extrapolating the weight on
    // the missing load cells
    if (online_load_cells() > 1) {
        total_weight = 0;
        for (int i = 0; i < total_load_cells; i++) {
            if (connected[i]) {
                total_weight += data[i];
            }
        }
        output_topic->data[0] = total_weight;
        // Update diagnostic topic
    } else {
        output_topic->data[0] = 0;
        // Update diagnostic topic
    }

}

void LoadCells::subscribe(ros::NodeHandle *node_handle) {
    node_handle->subscribe(setpoint_sub);
}

void LoadCells::tare() {
    for (int i = 0; i < total_load_cells; i++) {
        if (connected[i]) {
            load_cells[i]->tare();
        }
    }
    // Get the offset of each load cell and store it in EEPROM so that it can be retrieved on startup
    for (int i = 0; i < total_load_cells; i++) {
        int32_t offset = load_cells[i]->get_offset();
        EEPROM.put(EEPROM_CALIBRATION_ADDRESS_START + i * sizeof(int32_t), offset);
    }
}

void LoadCells::message_callback(const std_msgs::Int32MultiArray &msg) {
    // This is a very simple callback, if a message is ever received it just tares the load cells
    tare();
}
