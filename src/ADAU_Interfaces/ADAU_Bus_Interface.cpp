//
// Created by Jay on 10/31/2023.
//

#include "ADAU_Bus_Interface.h"

extern ADAU_Bus_Interface ADAU_BUS_INTERFACE;

void ADAU_Bus_Interface::attachSensor(ADAU_Sensor *sensor) {
    // Add the sensor to the array of sensor pointers
    sensors[this->num_sensors] = sensor;
}

void ADAU_Bus_Interface::load_header() {
    while (ADAU_INTERFACE.available()){
        // Read the next byte
        uint8_t byte = ADAU_INTERFACE.read();
        this->header_buffer[this->header_length] = byte;
        this->header_length++;
        if (this->header_length == 3) {
            // We have received the entire header
            // Parse the header
            this->message_header.sensor_id = this->header_buffer[0];
            this->message_header.data_length = this->header_buffer[1];
            this->message_header.checksum = this->header_buffer[2];
            // Check if the data length is valid
            if (this->message_header.data_length > 254) {
                // The data length is invalid
                // Reset the header received flag
                this->header_received = false;
                // Increment the failed message count
                this->failed_message_count++;
                // Break out of the while loop
                break;
            }
            // The header is valid
            // Set the header received flag
            this->header_received = true;
            // Reset the header length
            this->header_length = 0;
            // Reset the message buffer length
            this->message_buffer_len = 0;
            // Break out of the while loop
            break;
        }
    }
    this->load_data();
}

void ADAU_Bus_Interface::load_data(){
    // If the header has been received keep reading bytes until we get all the expected bytes
    while (ADAU_INTERFACE.available()) {
        // Read the next byte
        uint8_t byte = ADAU_INTERFACE.read();
        // Add the byte to the message buffer
        this->message_buffer[this->message_buffer_len] = byte;
        // Increment the message buffer length
        this->message_buffer_len++;
        // If the message buffer length is equal to the expected data length then we have received all the data
        if (this->message_buffer_len == this->message_header.data_length) {
            // Finish the message
            this->finish_message();
            // Break out of the while loop
            break;
        }
    }
}

void ADAU_Bus_Interface::finish_message() {

}

void ADAU_Bus_Interface::parse_buffer() {
    while (ADAU_INTERFACE.available()) {
        if (this->header_received) {
            this->load_data();
        } else {
            this->load_header();
        }
    }
}