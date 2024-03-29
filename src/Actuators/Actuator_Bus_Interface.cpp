//
// Created by Jay on 3/16/2023.
//

#include "Actuator_Bus_Interface.h"
#include "Main_Helpers/BreadCrumbs.h"

Actuator_Bus_Interface ACTUATOR_BUS_INTERFACE = Actuator_Bus_Interface();

//Calculates CRC16 of nBytes of data in byte array serial_message
uint16_t Actuator_Bus_Interface::crc16(const uint8_t *packet, uint32_t nBytes) {
    uint16_t crc = 0;
    for (int byte = 0; byte < nBytes; byte++) {
        crc = crc ^ ((unsigned int)packet[byte] << 8);
        for (unsigned char bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Processes serial messages that do not contain data
 * @param msg The serial message to process
 */
void Actuator_Bus_Interface::process_no_data_serial(serial_message* msg){
    if (Serial2.available() >= 1) {
        DROP_CRUMB();
        this->total_messages_received++;
        if (Serial2.read() == 0xFF) {
            this->waiting_for_response = false;
            if (msg->object != nullptr) {
                // Call the callback function
                auto object = msg->object;
                auto callback = msg->callback; // Cast the callback to a function pointer
                callback(object, msg);
                if (msg->free_after_callback) delete msg;
                this->total_messages_processed++;
            }
        } else {
            // If the response is not successful, send the serial_message again
            this->waiting_for_response = false;
            msg->failed_crc = true;
            // Free the serial_message
            if (msg->failure_callback != nullptr) {
                auto object = msg->object;
                auto callback = msg->failure_callback; // Cast the callback to a function pointer
                callback(object, msg);
            }
            if (msg->callback) if (msg->free_after_callback) delete msg;
        }
    }
}

/**
 * @brief Processes serial messages that contain data
 * @param msg The serial message to process
 */
void Actuator_Bus_Interface::process_data_serial(serial_message *msg) {
    if (Serial2.available() == msg->data_length + sizeof(uint16_t)){
        DROP_CRUMB();
        this->total_messages_received++; // Increment the total messages received
        Serial2.readBytes(this->response_buffer, msg->data_length + sizeof(uint16_t));
        memcpy(msg->data, this->response_buffer, msg->data_length); // Copy the data into the serial_message
        uint8_t crc_buffer[128] = {0};
        // Copy the sent address and the command id into the first two bytes of the crc buffer
        crc_buffer[0] = msg->id;
        crc_buffer[1] = msg->command;
        // Copy the data into the crc buffer
        memcpy(crc_buffer + 2, msg->data, msg->data_length);
        // Calculate the CRC
        calc_crc = this->crc16(crc_buffer, msg->data_length + 2);
        // Get the CRC from the response
        crc = (uint16_t) (this->response_buffer[msg->data_length] << 8 | this->response_buffer[msg->data_length + 1]);
        if (crc == calc_crc) {
            // The response is signal_valid
            this->waiting_for_response = false;
            if (msg->object != nullptr && msg->callback != nullptr) {
                // Call the callback function
                auto object = msg->object;  // Get the messages actuator object
                auto callback = msg->callback; // Cast the callback to a function pointer
                callback(object, msg);  // Call the callback function with the object and the serial_message
                if (msg->free_after_callback) delete msg; // NEVER USE
                this->total_messages_processed++;
            }
        } else {
            // If the response fails the CRC, then call the failure callback
            this->waiting_for_response = false;
            msg->failed_crc = true;
            // Free the serial_message
            if (msg->failure_callback != nullptr) {
                auto object = msg->object;
                auto callback = msg->failure_callback; // Cast the callback to a function pointer
                callback(object, msg);
            }
            if (msg->callback) if (msg->free_after_callback) delete msg;
        }
    }
}

/**
 * @brief Checks for a response to a sent message
 */
void Actuator_Bus_Interface::check_for_response(){
    serial_message* msg = this->message_queue[this->message_queue_dequeue_position];
    if (msg == nullptr) return; // This should not be null, but just in case

    if(msg->expect_response) { // Determine if we are waiting for data or just a success serial_message
        this->process_data_serial(msg);   // This checks for a data serial_message
    } else {
        this->process_no_data_serial(msg);  // This checks for a success serial_message
    }
    if (this->waiting_for_response && ((millis() - this->last_message_sent_time) > 11)){
        // If we have waited more than 10ms second for a response, then we abort this serial_message and remove it
        // from the queue and call the failure callback
        this->waiting_for_response = false; // We are no longer waiting for a response
        auto object = msg->object;
        if (msg->failure_callback != nullptr) {
            auto callback = msg->failure_callback; // Cast the callback to a function pointer
            callback(object, msg);
            if (msg->free_after_callback) delete msg;
        }
        this->message_queue[this->message_queue_dequeue_position] = nullptr; // Remove the pointer from the queue
    }
    if (!this->waiting_for_response){
        this->last_message_round_trip = millis() - this->last_message_sent_time;
        this->last_message_sent_time = 0;
        // Update the average round trip time
        this->average_time_per_message = (this->average_time_per_message * 9 + this->last_message_round_trip) / 10;
        memset(this->response_buffer, 0, sizeof(this->response_buffer));
    }
}

/**
 * @brief Sends messages and checks for responses
 * @return True if a message was sent or a response was received, false otherwise
 */
boolean Actuator_Bus_Interface::spin() {
    if (this->waiting_for_response){
        this->check_for_response();
        return true;
    } else {
        if (this->spin_start_time == 0){
            this->spin_start_time = millis();
        }
        serial_message* next_message = this->get_next_message();
        if (next_message != nullptr){
            DROP_CRUMB_VALUE((uint32_t) next_message, breadcrumb_type::ADDRESS);
            // Drop the address of the next message ID
            this->sent_last_cycle++;
            // Clear the transmit buffer
            memset(this->transmit_buffer, 0, sizeof(this->transmit_buffer));
            // Clear the serial buffer
            Serial2.clear();
            // Send the serial_message
            this->transmit_buffer[0] = next_message->id;
            this->transmit_buffer[1] = next_message->command;
            memcpy(this->transmit_buffer + 2, next_message->data, next_message->data_length);
            if (next_message->protected_action){  // If this is a protected action, then we need to calculate the CRC
                uint16_t transmit_crc = this->crc16(this->transmit_buffer, next_message->data_length + 2);
                // Append the CRC to the serial_message (MSB first)
                this->transmit_buffer[next_message->data_length + 2] = (uint8_t) (transmit_crc >> 8);
                this->transmit_buffer[next_message->data_length + 3] = (uint8_t) (transmit_crc & 0xFF);
                Serial2.write(this->transmit_buffer, next_message->data_length + sizeof(transmit_crc) + 2);
            } else {
                Serial2.write(this->transmit_buffer, next_message->data_length + 2);
            }
            Serial2.flush();  // Wait for the serial_message to be sent
            this->total_messages_sent++;
            this->waiting_for_response = true;
            this->last_message_sent_time = millis();
            return true;
        } else {
            this->spin_total_time = millis() - this->spin_start_time;
            this->spin_start_time = 0;
            return false;
        }
    }
}

Actuator_Bus_Interface::serial_message *Actuator_Bus_Interface::get_next_message() {
    // The serial_message queue is a circular buffer
    if (this->message_queue_enqueue_position == this->message_queue_dequeue_position){
        return nullptr;
    } else {
        this->message_queue_dequeue_position++;
        if (this->message_queue_dequeue_position >= MESSAGE_QUEUE_SIZE){
            this->message_queue_dequeue_position = 0;
        }
        serial_message* msg = this->message_queue[this->message_queue_dequeue_position];
        return msg;
    }
}

void Actuator_Bus_Interface::queue_message(Actuator_Bus_Interface::serial_message *message) {
    if (!this->space_available()) {
        if (message->free_after_callback) delete message;  // NEVER USE
        return;
    }
    // Check if the pointer to this message is already in the queue
//    for (auto & i : this->message_queue){
//        if (i == message){
//            return; // If it is, then don't add it again because its the exact same serial_message
//        }
//    }
    this->message_queue_enqueue_position++;
    if (this->message_queue_enqueue_position >= MESSAGE_QUEUE_SIZE){
        this->message_queue_enqueue_position = 0;
    }
    this->message_queue[this->message_queue_enqueue_position] = message;

}

bool Actuator_Bus_Interface::space_available() const {
    if (this->message_queue_enqueue_position == this->message_queue_dequeue_position){
        return true;
    } else {
        int next_position = this->message_queue_enqueue_position + 1;
        if (next_position >= MESSAGE_QUEUE_SIZE){
            next_position = 0;
        }
        if (next_position == this->message_queue_dequeue_position){
            return false;
        } else {
            return true;
        }
    }
}

uint8_t Actuator_Bus_Interface::get_queue_size() const {
    // Calculate how many messages are in the queue
    if (this->message_queue_enqueue_position == this->message_queue_dequeue_position){
        return 0;
    } else if (this->message_queue_enqueue_position > this->message_queue_dequeue_position){
        return this->message_queue_enqueue_position - this->message_queue_dequeue_position;
    } else {
        return MESSAGE_QUEUE_SIZE - this->message_queue_dequeue_position + this->message_queue_enqueue_position;
    }
}

uint32_t Actuator_Bus_Interface::round_trip_time() const {
    return this->last_message_round_trip;
}
