//
// Created by Jay on 3/16/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ACTUATORS_H
#define TEENSYCANTRANSCEIVER_ACTUATORS_H

#include <Arduino.h>

class Actuators {

public:
    enum serial_commands: uint8_t {
        read_encoder_count_m1 = 16,
        read_encoder_count_m2 = 17,
        read_encoder_speed_m1 = 18,
        read_encoder_speed_m2 = 19,
        read_encoder_counts = 78,
        read_encoder_speeds = 79,
        set_encoder_count_m1 = 22,
        set_encoder_count_m2 = 23,
        read_main_battery_voltage = 24,
        read_logic_battery_voltage = 25,
        read_motor_pwms = 48,
        read_motor_currents = 49,
        read_temperature = 82,
        read_status = 90,
        drive_m1_duty_cycle = 32,
        drive_m2_duty_cycle = 33,
        set_position_m1 = 65,
        set_position_m2 = 66,
        set_positions = 67,
    };


    struct message{
        uint8_t id;
        serial_commands command;
        uint8_t data_length;
        uint8_t data[16];
        uint8_t crc[2];
        bool expect_response;  // If true the object will send data back
        bool sent_received;    // True if the message has been sent and a response has been received
        // The callback function to call when the message is received
        void* object;
        void (*callback)(void* actuator, message* msg);
        boolean free_after_callback = false; // If true the message will be deleted after the callback is called
        void (*failure_callback)(void* actuator, message* msg); // The callback to call if the message fails to send
    };

private:

    // The message queue is a buffer for having object objects send messages to their respective actuators


    message* message_queue[20] = {nullptr};
    uint8_t message_queue_enqueue_position = 0;
    uint8_t message_queue_dequeue_position = 19;
    boolean waiting_for_response = false;

    // Variables used for generating debug information
    uint32_t last_message_sent_time = 0;
    uint32_t last_message_round_trip = 0;
    uint32_t spin_start_time = 0;


    uint16_t crc16(const uint8_t *packet, uint32_t nBytes);

    message* get_next_message();

    void process_no_data_serial(message* msg);

    void process_data_serial(message* msg);

    uint8_t get_queue_size() const;

public:

    // Debug variables
    uint32_t average_time_per_message = 0;
    uint32_t message_count = 0;
    uint32_t spin_total_time = 0;

    Actuators(){
        Serial1.begin(115200);
        for (auto & i : message_queue){
            i = nullptr;
        }
    }

    void queue_message(message *message);

    bool space_available() const;

    /**
    * Sendsd messages to the actuators from the message queue
    * @param lastSpin - True if this is the last time the spin function will be called
    * @return True if there are more messages to send
    */
    boolean spin(boolean lastSpin);

    String* get_status_string();

};


#endif //TEENSYCANTRANSCEIVER_ACTUATORS_H
