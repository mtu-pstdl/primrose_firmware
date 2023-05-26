//
// Created by Jay on 5/26/2023.
//

#include "LoadCells.h"


void LoadCells::publish() {

}

void LoadCells::update() {
    // Read all connected load cells
    for (int i = 0; i < load_cell_number; i++) {
        if (connected[i]) {
            output_topic->data[i + 1] = load_cells[i]->read();
            // Update diagnostic topic
            sprintf(value_strings[i + 1], "%ld", output_topic->data[i + 1]);
        } else {
            output_topic->data[i + 1] = 0;
            // Update diagnostic topic
            sprintf(value_strings[i + 1], "Not Connected");
        }
    }
    // If more than 1 load cell is connected, calculate the total weight by extrapolating the weight on
    // the missing load cells
    if (online_load_cells() > 1) {
        int32_t total_weight = 0;
        for (int i = 0; i < load_cell_number; i++) {
            if (connected[i]) {
                total_weight += output_topic->data[i + 1];
            }
        }
        output_topic->data[0] = total_weight;
        // Update diagnostic topic
        sprintf(value_strings[0], "%ld", output_topic->data[0]);
    } else {
        output_topic->data[0] = 0;
        // Update diagnostic topic
        sprintf(value_strings[0], "No Valid Data Points");
    }

}

void LoadCells::subscribe(ros::NodeHandle *node_handle) {
    node_handle->subscribe(setpoint_sub);
}