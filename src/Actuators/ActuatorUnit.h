//
// Created by Jay on 3/16/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ACTUATORUNIT_H
#define TEENSYCANTRANSCEIVER_ACTUATORUNIT_H

#include <Arduino.h>
#include "Actuators.h"

class ActuatorUnit {

public:

    static void encoder_count_callback(void* actuator, Actuators::message* msg);

    static void encoder_speed_callback(void* actuator, Actuators::message* msg);

    static void main_battery_voltage_callback(void* actuator, Actuators::message* msg);

    static void logic_battery_voltage_callback(void* actuator, Actuators::message* msg);

private:

    Actuators* command_bus;

    uint8_t id;

    struct telemetry_message{
        Actuators::message* msg; // The message to send
        uint32_t last_send_time; // The time the message was last sent
        uint32_t send_interval;  // The minimum time between sending the message
    };

    telemetry_message* reocurring_messages;

    int32_t encoder_pos_m1; // The absolute value of the encoder position
    boolean  negative_value_m1; // True if the encoder position is negative
    int32_t encoder_pos_m2; // The absolute value of the encoder position
    boolean  negative_value_m2; // True if the encoder position is negative

    int32_t encoder_speed_m1; // The absolute value of the encoder speed in encoder ticks per second
    int32_t encoder_speed_m2; // The absolute value of the encoder speed in encoder ticks per second

    uint16_t main_battery_voltage; // The main battery voltage in tenths of a volt
    uint16_t logic_battery_voltage; // The logic battery voltage in tenths of a volt

    uint32_t current_m1; // The current in tenths of an amp
    uint32_t current_m2; // The current in tenths of an amp

    void queue_messages();

    telemetry_message* build_message(Actuators::serial_commands command, uint32_t send_interval, uint8_t data_length,
                                     void (*callback)(void *, Actuators::message*));

    void build_telemetry_messages();

public:


    ActuatorUnit(Actuators* command_bus, uint8_t id) {
        this->command_bus = command_bus;
        this->id = id;

        // Setup all the required messages for gathering information from the object
        this->reocurring_messages = new telemetry_message[10];

    }




};


#endif //TEENSYCANTRANSCEIVER_ACTUATORUNIT_H
