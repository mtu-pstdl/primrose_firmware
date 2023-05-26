//
// Created by Jay on 5/26/2023.
//

#ifndef TEENSYCANTRANSCEIVER_LOADCELL_H
#define TEENSYCANTRANSCEIVER_LOADCELL_H

#include <Arduino.h>
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/UInt32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/KeyValue.h"

class LoadCell {

    int pin_number;
    int load_cell_number;
    int calibration_factor;
    int calibration_offset;
    diagnostic_msgs::DiagnosticStatus* status;
    std_msgs::UInt32MultiArray* output_topic;

    LoadCell(int pin_number, int load_cell_number, int calibration_factor, int calibration_offset,
             diagnostic_msgs::DiagnosticStatus* status, std_msgs::UInt32MultiArray* output_topic){
        this->pin_number = pin_number;
        this->load_cell_number = load_cell_number;
        this->calibration_factor = calibration_factor;
        this->calibration_offset = calibration_offset;
        this->status = status;
        this->output_topic = output_topic;
        this->output_topic->data_length = 2;
        this->output_topic->data = new uint32_t[2];

        pinMode(pin_number, INPUT);
    }

    void read_load_cell(){
        int raw_value = analogRead(pin_number);
        int calibrated_value = raw_value * calibration_factor + calibration_offset;
        output_topic->data[0] = load_cell_number;
        output_topic->data[1] = calibrated_value;
    }

};


#endif //TEENSYCANTRANSCEIVER_LOADCELL_H
