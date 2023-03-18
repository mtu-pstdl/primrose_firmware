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

boolean Actuators::spin(boolean lastSpin) {
    if (this->waiting_for_response){
        message* msg = this->message_queue[this->message_queue_dequeue_position];
        if(!msg->expect_response) {
            this->process_no_data_serial(msg);
        } else {
            this->process_data_serial(msg);
        }
        return true;
    } else {
        if (lastSpin){  // If we have run out of spin time, then don't send another message
            return false;
        }
        message* next_message = this->get_next_message();
        if (next_message != nullptr){
            // Send the message
            Serial1.write(next_message->id);
            Serial1.write(next_message->command);
            Serial1.write(next_message->data_length);
            Serial1.write(next_message->data, next_message->data_length);
            Serial1.write(next_message->crc, 2);
            this->waiting_for_response = true;
            return true;
        } else {
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
