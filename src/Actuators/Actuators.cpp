//
// Created by Jay on 3/16/2023.
//

#include "Actuators.h"



//Calculates CRC16 of nBytes of data in byte array message
uint16_t Actuators::crc16(const uint8_t *packet, uint32_t nBytes) {
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

void Actuators::process_no_data_serial(message* msg){
    if (Serial1.available() >= 1) {
        uint8_t response[1];
        Serial1.readBytes(response, 1);
        if (response[0] == 0xFF) {
            this->waiting_for_response = false;
            if (msg->object != nullptr) {
                // Call the callback function
                auto object = msg->object;
                auto callback = msg->callback; // Cast the callback to a function pointer
                callback(object, msg);
                if (msg->free_after_callback) delete msg;
            }
        } else {
            // If the response is not successful, send the message again
            this->message_queue_dequeue_position--;
            if (this->message_queue_dequeue_position < 0) {
                this->message_queue_dequeue_position = 19;
            }
            this->waiting_for_response = false;
        }
    }
}

void Actuators::process_data_serial(message *msg) {
    if (Serial1.available() >= msg->data_length + sizeof(msg->crc)){
        uint8_t response[msg->data_length + sizeof(msg->crc)];
        Serial1.readBytes(response, msg->data_length + sizeof(msg->crc));
        uint16_t crc = this->crc16(response, msg->data_length + sizeof(msg->crc));
        memcpy(msg->data, response, msg->data_length); // Copy the data into the message
        if (crc == 0){
            // The response is valid
            this->waiting_for_response = false;
            if (msg->object != nullptr && msg->callback != nullptr) {
                // Call the callback function
                auto object = msg->object;
                auto callback = msg->callback; // Cast the callback to a function pointer
                callback(object, msg);
                if (msg->free_after_callback) delete msg;
            }
        } else {
            // If the response is not successful, send the message again
            this->message_queue_dequeue_position--;
            if (this->message_queue_dequeue_position < 0) {
                this->message_queue_dequeue_position = 19;
            }
            this->waiting_for_response = false;
        }
    }
}

void Actuators::check_for_response(){
    message* msg = this->message_queue[this->message_queue_dequeue_position];
    if(!msg->expect_response) {
        this->process_no_data_serial(msg);
    } else {
        this->process_data_serial(msg);
    }
    if (this->waiting_for_response && ((millis() - this->last_message_sent_time) > 10)){
        // If we have waited more than 10ms second for a response, then we abort this message and remove it
        // from the queue
        this->waiting_for_response = false;
        auto object = msg->object;
        if (msg->failure_callback != nullptr) {
            auto callback = msg->failure_callback; // Cast the callback to a function pointer
            callback(object, msg);
            if (msg->free_after_callback) delete msg;
        }
        this->message_queue[this->message_queue_dequeue_position] = nullptr;
    }
    if (!this->waiting_for_response){
        this-> last_message_round_trip = millis() - this->last_message_sent_time;
        this->last_message_sent_time = 0;
        // Update the average round trip time
        this->average_time_per_message = (this->average_time_per_message * 9 + this->last_message_round_trip) / 10;
    }
}

boolean Actuators::spin(boolean lastSpin) {
    if (this->waiting_for_response){
        this->check_for_response();
        return true;
    } else {
        if (this->spin_start_time == 0){
            this->spin_start_time = millis();
        }
        if (lastSpin){  // If we have run out of spin time, then don't send another message
            this->spin_total_time = millis() - this->spin_start_time;
            this->spin_start_time = 0;
            return false;
        }
        message* next_message = this->get_next_message();
        if (next_message != nullptr){
            // Send the message
            uint8_t data[next_message->data_length + sizeof(next_message->crc) + 2];
            data[0] = next_message->id;
            data[1] = next_message->command;
            memcpy(data + 2, next_message->data, next_message->data_length);
            uint16_t crc = this->crc16(data, next_message->data_length + 2);
            memcpy(data + next_message->data_length + 2, &crc, sizeof(crc));
            Serial1.write(data, next_message->data_length + sizeof(crc) + 2);
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

Actuators::message *Actuators::get_next_message() {
    // The message queue is a circular buffer
    if (this->message_queue_enqueue_position == this->message_queue_dequeue_position){
        return nullptr;
    } else {
        this->message_queue_dequeue_position++;
        if (this->message_queue_dequeue_position >= 20){
            this->message_queue_dequeue_position = 0;
        }
        return this->message_queue[this->message_queue_dequeue_position];
    }
}

void Actuators::queue_message(Actuators::message *message) {
    if (!this->space_available()) {
        free(message);
        return;
    }
    this->message_queue_enqueue_position++;
    if (this->message_queue_enqueue_position >= 20){
        this->message_queue_enqueue_position = 0;
    }
    this->message_queue[this->message_queue_enqueue_position] = message;

}

bool Actuators::space_available() const {
    if (this->message_queue_enqueue_position == this->message_queue_dequeue_position){
        return true;
    } else {
        int next_position = this->message_queue_enqueue_position + 1;
        if (next_position >= 20){
            next_position = 0;
        }
        if (next_position == this->message_queue_dequeue_position){
            return false;
        } else {
            return true;
        }
    }
}

uint8_t Actuators::get_queue_size() const {
    // Calculate how many messages are in the queue
    if (this->message_queue_enqueue_position == this->message_queue_dequeue_position){
        return 0;
    } else if (this->message_queue_enqueue_position > this->message_queue_dequeue_position){
        return this->message_queue_enqueue_position - this->message_queue_dequeue_position;
    } else {
        return 20 - this->message_queue_dequeue_position + this->message_queue_enqueue_position;
    }
}

uint32_t Actuators::round_trip_time() const {
    return this->last_message_round_trip;
}

String* Actuators::get_status_string() {
    auto* status_string = new String();
    status_string->concat("\r\n-------------Actuator bus status------------\r\n");
    status_string->concat("Average round trip time: " + String(this->average_time_per_message) + "ms\r\n");
    status_string->concat("Last round trip time: " + String(this->last_message_round_trip) + "ms\r\n");
    status_string->concat("Messages in queue: " + String(this->get_queue_size()) + "\r\n");
    status_string->concat("Waiting for response: " + String(this->waiting_for_response) + "\r\n");
    status_string->concat("Spin time: " + String(this->spin_total_time) + "ms\r\n");
    return status_string;
}
