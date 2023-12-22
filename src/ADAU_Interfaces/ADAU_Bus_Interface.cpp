//
// Created by Jay on 10/31/2023.
//

#include "ADAU_Bus_Interface.h"
#include "ADAU_Sensor.h"
#include "Main_Helpers/BreadCrumbs.h"

/**
 * @brief The default constructor for the ADAU_Bus_Interface class
 * @warning The ADAU_Bus_Interface class is a singleton and should not be instantiated directly
 * @warning This object should only be accessed by the ADAU_SENSOR class and should not be used directly
 */
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

/**
 * @brief  Load the first 3 bytes of the message which is the header
 * @details The header contains the sensor id, data length, and checksum
 *          If this process is interrupted due to either a timeout or the buffer being empty it will just pick up
 *          where it left off next time it is called
 * @previous_step waiting_for_start_byte or waiting_for_header
 * @next_step prevalidate_data_length()
 */
void ADAU_Bus_Interface::load_header() {
    while (ADAU_INTERFACE.available() && this->parse_start_time + MAX_PARSE_TIME > micros()) {
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
    this->prevalidate_data_length();
}

/**
 * @brief  Check if the data length matches the expected data length of the sensor it supposedly is for
 * @details If the data length does not match the expected data length then we have received either a corrupted message
 *          or a message from a sensor that we did not expect to exist and therefor have no way to decode
 * @previous_step load_header()
 * @next_step load_data()
 */
void ADAU_Bus_Interface::prevalidate_data_length() {
    // Check if the data length matches the expected data length of the sensor it supposedly is for
    ADAU_Sensor_List* current = this->sensor_list;
    boolean success = false;
    while (current != nullptr) {
        if (current->sensor == nullptr) continue;  // Skip this sensor if it is null
        if (current->sensor->get_sensor_id() == this->message_header.sensor_id) {
            // We have found the sensor that sent the message
            // Update the sensor
            // Check if the checksum is signal_valid
            if (this->message_header.data_length != current->sensor->get_data_size()) {
                break;
            }
            success = true;
        }
        if (current->next == nullptr) break;
        current = current->next;
    }
    if (!success) {
        // We did not find the sensor that sent the message
        // Increment the failed message count
        this->current_state = waiting_for_end_byte_failure;
        this->failed_message_count++;
    } else {
        // We found the sensor that sent the message
        // Set the current state to waiting for data
        this->current_state = waiting_for_data;
    }
}

/**
 * @brief  Load the data bytes of the message into the message buffer
 * @details If this process is interrupted due to either a timeout or the buffer being empty it will just pick up
 *          where it left off next time it is called
 * @previous_step prevalidate_data_length()
 * @next_step finish_message()
 */
void ADAU_Bus_Interface::load_data(){
    // If the header has been received keep reading bytes until we get all the expected bytes
    if (!ADAU_INTERFACE.available()) return;
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
    }
}

/**
 * @brief  Finish the message by reading the trailing end message bytes
 * @details If this process is interrupted due to either a timeout or the buffer being empty it will just pick up
 *          where it left off next time it is called
 * @previous_step load_data()
 * @next_step process_message()
 */
void ADAU_Bus_Interface::finish_message() {
    // When this function is called the message buffer should contain the entire message
    if (!ADAU_INTERFACE.available()) return;
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

/**
 * @brief  Process the message by validating the checksum and then updating the sensors memory location
 * @details If the checksum is invalid then the message is discarded
 * @previous_step finish_message()
 * @next_step cleanup()
 */
void ADAU_Bus_Interface::process_message() {
    // Find the sensor that sent the message

    // Validate the checksum
    if (!this->validate_checksum()) {
        // The checksum is invalid
        // Increment the failed message count
        this->failed_message_count++;
        this->cleanup();
        return;
    }
    boolean found_sensor = false;
    ADAU_Sensor_List* current = this->sensor_list;
    while (current != nullptr) {
        if (current->sensor == nullptr) continue;  // Skip this sensor if it is null
        if (current->sensor->get_sensor_id() == this->message_header.sensor_id) {
            // We have found the sensor that sent the message
            // Update the sensor
            // Check if the checksum is signal_valid
            if (this->message_header.data_length != current->sensor->get_data_size()) {
                // The data length is invalid
                // Increment the failed message count
                this->failed_message_count++;
                continue;
            }
            void* data_ptr = current->sensor->get_data_ptr();
            memcpy(data_ptr, this->message_buffer, this->message_header.data_length);
            current->sensor->set_last_update_time(micros());
            current->sensor->set_valid(true);
            // Record the time that the last message was received
            this->last_message_time = micros();
            parse_count++;
            found_sensor = true;
        }
        if (current->next == nullptr) break;
        current = current->next;
    }
    if (!found_sensor) this->failed_message_count++;
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
 * @brief  Cleanup the bus interface by resetting all the variables and flags
 * @details This function is called after a message has been processed or if the message is invalid
 * @previous_step process_message() or finish_message()
 * @next_step waiting_for_start_byte
 */
void ADAU_Bus_Interface::cleanup() {
    // Reset the header received flag
    this->current_state = waiting_for_start_byte;
    // Reset the message end count
    this->message_end_count = 0;
    // Reset the header length
    this->header_length = 0;
    // Reset the message buffer length
    this->message_buffer_len = 0;
    // Reset the header buffer
    memset(this->header_buffer, 0, 3);
    // Reset the message buffer
    memset(this->message_buffer, 0, 254);
}

/**
 * @brief  Parse the serial buffer and update the sensors
 * @details This function is called in the main loop and will parse the serial buffer and update the sensors
 *          as long as there is time or data left to parse
 * @warning This function will block the main loop for up to 2.5 ms while it parses the serial buffer
 * @warning If the buffer is not cleared fast enough then the buffer will overflow and data will be lost
 */
void ADAU_Bus_Interface::parse_buffer() {
    DROP_CRUMB();
    this->parse_start_time = micros(); // Record the time that the parse started so we don't overrun the allotted time
    // Print the contents of the buffer to the temp string in hex
    this->parse_count = 0;
    this->failed_message_count = 0;
    uint32_t ignored_bytes = 0;
    memset(this->output_string, 0, 999);
    sprintf(this->output_string, "Starting parse, entry state: %d, buffer length: %d\n",
            this->current_state, ADAU_INTERFACE.available());
    while (this->parse_start_time + MAX_PARSE_TIME > micros()) {
        if (!ADAU_INTERFACE.available()) break;
        switch (this->current_state) {
            case waiting_for_start_byte:
                // Wait for the start byte
                ignored_bytes++;
                if (ADAU_INTERFACE.read() == MESSAGE_START_BYTE) {
                   // We have received the start byte
                   // Set the current state to waiting for header
                   this->current_state = waiting_for_header;
                   ignored_bytes--;
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
           default:
               this->cleanup();
       }
    }
    sprintf(this->output_string, "%sFinished parse, exit state: %d, buffer length: %d,"
                                 " parse count: %d, failed count: %lu, ignored bytes: %lu, "
                                 "time elapsed: %lu us\n",
            this->output_string, this->current_state, ADAU_INTERFACE.available(),
            this->parse_count, this->failed_message_count, ignored_bytes,
            micros() - this->parse_start_time);
}