//
// Created by Jay on 10/31/2023.
//

#include "ADAU_Bus_Interface.h"
#include "ADAU_Sensor.h"

ADAU_Bus_Interface ADAU_BUS_INTERFACE = ADAU_Bus_Interface();

/**
 * @brief Attach a sensor to the bus interface by adding it to the linked sensor list
 * @param sensor A pointer to the sensor object to attach
 */
void ADAU_Bus_Interface::attachSensor(ADAU_Sensor *sensor) {
    // Add the sensor to the sensor list
    this->num_sensors++;
    ADAU_Sensor_List* current = this->sensor_list;
    if (current == nullptr) {
        // The sensor list is empty
        this->sensor_list = new ADAU_Sensor_List;
        this->sensor_list->sensor = sensor;
    } else {
        // The sensor list is not empty
        while (true) {
            if (current->next == nullptr) {
                // We have reached the end of the list
                current->next = new ADAU_Sensor_List;
                current->next->sensor = sensor;
                break;
            }
            current = current->next;
        }
    }
}

void ADAU_Bus_Interface::load_header() {
    while (ADAU_INTERFACE.available() && this->parse_start_time + 250 > micros()) {
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
            // Check if the data length is signal_valid
            if (this->message_header.data_length > 254) {
                // The data length is invalid
                // Reset the header received flag
                this->current_state = waiting_for_end_byte_failure;
                // Increment the failed message count
                this->failed_message_count++;
                // Break out of the while loop
                break;
            }
            // The header is signal_valid
            this->current_state = waiting_for_data;
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
    while (ADAU_INTERFACE.available() && this->parse_start_time + 250 > micros()) {
        // Read the next byte
        uint8_t byte = ADAU_INTERFACE.read();
        // Add the byte to the message buffer
        this->message_buffer[this->message_buffer_len] = byte;
        // Increment the message buffer length
        this->message_buffer_len++;
        // If the message buffer length is equal to the expected data length then we have received all the data
        if (this->message_buffer_len == this->message_header.data_length) {
            this->current_state = waiting_for_end_byte;
            // Finish the message
            this->finish_message();
            // Break out of the while loop
            break;
        }
    }
}

void ADAU_Bus_Interface::finish_message() {
    // When this function is called the message buffer should contain the entire message
    // Wait for the end of message bytes
    while (ADAU_INTERFACE.available() && this->parse_start_time + 250 > micros()) {
        uint8_t byte = ADAU_INTERFACE.read();
        if (byte == MESSAGE_END_BYTE) {
            this->message_end_count++;
        } else {
            this->message_end_count = 0;
        }
        if (this->current_state == waiting_for_end_byte) {
            if (this->message_end_count == END_MESSAGE_COUNT) {
                // We have received the footer so now we can process the message
                this->process_message();
            }
        } else if (this->current_state == waiting_for_end_byte_failure) {
            if (this->message_end_count == END_MESSAGE_COUNT) {
                // We have received the footer so now we can cleanup
                this->cleanup();
            }
        }
    }
}

void ADAU_Bus_Interface::process_message() {
    // Check if the message is signal_valid
    parse_count = 0;
    if (this->validate_parity() || true) { // Disable parity check for now
        this->message_header.sensor_id >>= 1;  // Shift the sensor id right by 1 bit to remove the parity bit
        // The message is signal_valid
        // Find the sensor that sent the message
        ADAU_Sensor_List* current = this->sensor_list;
        while (current != nullptr) {
            if (current->sensor == nullptr) continue;  // Skip this sensor if it is null
            parse_count++;
            current->sensor->is_attached_properly();
            if (current->sensor->get_sensor_id() == this->message_header.sensor_id) {
                // We have found the sensor that sent the message
                // Update the sensor
                // Check if the checksum is signal_valid
                if (!this->validate_checksum()) {
                    // The checksum is invalid
                    // Increment the failed message count
                    current->sensor->set_valid(false);
                    this->failed_message_count++;
                    continue;
                }
                void* data_ptr = current->sensor->get_data_ptr();
                memcpy(data_ptr, this->message_buffer, this->message_header.data_length);
                current->sensor->set_last_update_time(millis());
                current->sensor->set_valid(true);
                // Record the time that the last message was received
                this->last_message_time = millis();
            }
//            if (current->next == nullptr) break;
            current = current->next;
        }
    } else {  // Because the sensor_id is invalid we can't find the sensor that sent the message, so we can't update it
        // The message is invalid
        // Increment the failed message count
        this->failed_message_count++;
    }
    this->cleanup();
}

/**
 * @brief  Calculate the checksum for a given sensor id and data
 * @details The checksum is calculated by adding the sensor id to the data length and then adding all the data bytes
 *          which wraps around at 255
 * @return checksum The checksum of the message
 */
boolean ADAU_Bus_Interface::validate_checksum() {
    uint8_t checksum = 0;
//    checksum += this->message_header.sensor_id;
    checksum += this->message_header.data_length;
    for(int i = 0; i < this->message_header.data_length; i++){
        checksum += this->message_buffer[i];
    }
    return checksum == this->message_header.checksum;
}

/**
 * @brief  Calculate if the parity for a given sensor id is valid (even parity, LSB is parity bit)
 * @return True if the parity is signal_valid, false if the parity is invalid
 */
boolean ADAU_Bus_Interface::validate_parity() {
    // Check if the sensor id passes the parity check
    uint8_t parity = 0;
    for (int i = 0; i < 7; i++) {
        parity += (this->message_header.sensor_id >> i) & 0x01;  // Add the value of the current bit to the parity
    }
    return parity % 2 == 0;
}

void ADAU_Bus_Interface::cleanup() {
    // Reset the header received flag
    this->current_state = waiting_for_start_byte;
    // Reset the message end count
    this->message_end_count = 0;
    // Reset the header length
    this->header_length = 0;
    // Reset the message buffer length
    this->message_buffer_len = 0;
}

void ADAU_Bus_Interface::parse_buffer() {
    this->parse_start_time = micros(); // Record the time that the parse started so we don't overrun the allotted time
    while (ADAU_INTERFACE.available() && this->parse_start_time + 250 > micros()) {
       switch (this->current_state) {
           case waiting_for_start_byte:
               // Wait for the start byte
               if (ADAU_INTERFACE.read() == MESSAGE_START_BYTE) {
                   // We have received the start byte
                   // Set the current state to waiting for header
                   this->current_state = waiting_for_header;
               }
               break;
           case waiting_for_header:
               // Wait for the header
               this->load_header();
               break;
           case waiting_for_data:
               // Wait for the data
               this->load_data();
               break;
           case waiting_for_end_byte:
           case waiting_for_end_byte_failure:
               // Wait for the end byte
               this->finish_message();
               break;
       }
    }
}