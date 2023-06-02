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

    for (int i = 2; i < total_load_cells + 2; i++) {
        if (connected[i-2]) {
            sprintf(value_strings[i], "%ld kg", data[i - 2]);
        } else {
            sprintf(value_strings[i], "Unavailable");
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
    node_handle->subscribe(control_sub);
}

void LoadCells::tare() {

    for (int i = 0; i < total_load_cells; i++) {
        if (connected[i]) {
            load_cells[i]->tare();
        } else { // Attempt to connect to the load cell
            if (this->load_cells[i]->wait_ready_timeout(250)){
                this->connected[i] = false;
            } else {
                this->connected[i] = true;
            }
        }
    }
    // Get the offset of each load cell and store it in EEPROM so that it can be retrieved on startup
    for (int i = 0; i < total_load_cells; i++) {
        set_offset(i, load_cells[i]->get_offset());
    }

    // Update diagnostic topic to reflect the new offsets
    for (int i = total_load_cells + 2; i < (total_load_cells * 2) + 2; i++) {
        sprintf(value_strings[i], "%ld kg", get_offset(i - total_load_cells - 2));
    }

    // Calculate the total offset
    int32_t total_offset = 0;
    for (int i = 0; i < total_load_cells; i++) {
        total_offset += get_offset(i);
    }
    sprintf(value_strings[1], "%ld kg", total_offset);

    // Update the diagnostic topic to indicate that the load cells are tared
    this->update_diagnostics_topic();
}

void LoadCells::message_callback(const std_msgs::Int32MultiArray &msg) {
    // This is a very simple callback, if a message is ever received it just tares the load cells
    tare();
}