//
// Created by Jay on 3/16/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ACTUATORS_H
#define TEENSYCANTRANSCEIVER_ACTUATORS_H

#include <Arduino.h>

#define MESSAGE_QUEUE_SIZE 100

class Actuator_Bus_Interface {

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
        read_buffer_length = 47,
        drive_m1_duty_cycle = 32,
        drive_m2_duty_cycle = 33,
        set_position_m1 = 65,
        set_position_m2 = 66,
        set_positions = 67,
        set_velocity_pid_gains_m1 = 28,
        set_velocity_pid_gains_m2 = 29,
    };


    struct serial_message {
        uint8_t id                  = 0;        // The address of the actuator
        serial_commands command     = read_encoder_count_m2;  // The command to send to the actuator
        uint16_t data_length        = 0;        // The length of the data to send
        uint16_t response_length    = 0;        // The length of the response to expect
        uint8_t data[24]            = {0};
        bool expect_response        = false;    // If true the actuator will send data back
        bool protected_action       = false;    // If true the serial_message will be sent with a crc
        bool failed_crc             = false;    // If true the serial_message failed the crc check
        // The callback function to call when the serial_message is received
        void* object                = nullptr;  // The pointer to the object that sent the serial_message
        void (*callback)(void* actuator, serial_message* msg) = nullptr; // The pointer to the callback function
        boolean free_after_callback = false; // If true the serial_message will be deleted after the callback is called
        // The callback to call if the serial_message fails to send
        void (*failure_callback)(void* actuator, serial_message* msg) = nullptr;
    };


private:

    // The serial_message queue is a buffer for having object objects send messages to their respective actuators


    serial_message* message_queue[MESSAGE_QUEUE_SIZE] = {nullptr};
    uint8_t response_buffer[128] = {0};
    uint8_t transmit_buffer[128] = {0};
    uint8_t write_buffer[128]    = {0};
    uint8_t message_queue_enqueue_position = 0;
    uint8_t message_queue_dequeue_position = 19;
    boolean waiting_for_response = false;

    // Variables used for generating debug information
    uint32_t last_message_sent_time = 0;
    uint32_t last_message_round_trip = 0;
    uint32_t spin_start_time = 0;


    static uint16_t crc16(const uint8_t *packet, uint32_t nBytes);

    serial_message* get_next_message();

    void check_for_response();

    void process_no_data_serial(serial_message* msg);

    void process_data_serial(serial_message* msg);

public:

    // Debug variables
    uint32_t average_time_per_message = 0;
    uint32_t message_count = 0;
    uint32_t spin_total_time = 0;

    uint32_t total_messages_sent = 0;
    uint32_t total_messages_received = 0;
    uint32_t total_messages_processed = 0;

    uint32_t sent_last_cycle = 0;

    uint16_t crc = 0;
    uint16_t calc_crc = 0;

    Actuator_Bus_Interface(){
        Serial2.begin(100000);
        // Yellow to brown
        // Green to purple
        Serial2.addMemoryForWrite(write_buffer, 128);
        Serial2.setTimeout(1000);
        for (auto & i : message_queue){
            i = nullptr;
        }
    }

    void queue_message(serial_message *message);

    bool space_available() const;

    uint8_t get_queue_size() const;

    uint32_t round_trip_time() const;

    /**
     * @brief This method should be called in the main loop of the program to process messages
     * @details This message sends messages to the actuators and then checks for responses
     * @return True messages remain in the queue
     */
    boolean spin();

};


#endif //TEENSYCANTRANSCEIVER_ACTUATORS_H
