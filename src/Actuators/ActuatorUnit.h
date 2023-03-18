//
// Created by Jay on 3/16/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ACTUATORUNIT_H
#define TEENSYCANTRANSCEIVER_ACTUATORUNIT_H

#include <Arduino.h>
#include "Actuators.h"

class ActuatorUnit {

public:

    enum controller_status_bitmask: uint16_t {
        normal                   = 0x0000,
        m1_over_current          = 0x0001,
        m2_over_current          = 0x0002,
        e_stop                   = 0x0004,
        high_temperature_fault   = 0x0008,
        main_battery_high_fault  = 0x0010,
        logic_battery_high_fault = 0x0020,
        logic_battery_low_fault  = 0x0040,
        m1_driver_fault          = 0x0080,
        m2_driver_fault          = 0x0100,
        main_battery_high_warn   = 0x0200,
        main_battery_high_low    = 0x0400,
        high_temperature_warn    = 0x0800,
        m1_homing                = 0x1000,
        m2_homing                = 0x2000,
    };

    enum control_modes {
        stopped,
        position,
        speed,
        homing
    };

    struct motor_info{
        uint32_t target_position     = 0; // The target position of the motor in tenths of a degree
        uint32_t current_position    = 0; // The current position of the motor in tenths of a degree
        boolean  position_negative   = false; // The current direction of the motor
        uint32_t current_speed       = 0; // The current speed of the motor in tenths of a degree per second
        boolean  direction_negative  = false; // The current direction of the motor
        uint16_t current_current     = 0; // The current current draw of the motor in tenths of an amp
        control_modes control_mode   = stopped; // The current control mode of the motor
        boolean  homed               = false; // Whether or not the motor has been homed
    };

    static void detailed_encoder_count_callback(void* actuator, Actuators::message* msg);

    static void encoder_count_callback(void* actuator, Actuators::message* msg);

    static void encoder_speed_callback(void* actuator, Actuators::message* msg);

    static void motor_currents_callback(void* actuator, Actuators::message* msg);

    static void main_battery_voltage_callback(void* actuator, Actuators::message* msg);

    static void logic_battery_voltage_callback(void* actuator, Actuators::message* msg);

    static void controller_status_callback(void* actuator, Actuators::message* msg);

private:

    Actuators* command_bus;

    uint8_t id;


    motor_info motors[2]{
            motor_info(),
            motor_info()
    };

    struct telemetry_message{
        Actuators::message* msg; // The message to send
        uint32_t last_send_time; // The time the message was last sent
        uint32_t send_interval;  // The minimum time between sending the message
    };

    telemetry_message* reocurring_messages;

    // Shared variables between motor 1 and motor 2
    uint16_t controller_temperature = 0;
    uint16_t main_battery_voltage = 0;
    uint16_t logic_battery_voltage = 0;
    uint16_t status = 0;

    telemetry_message* build_message(Actuators::serial_commands command, uint32_t send_interval, uint8_t data_length,
                                     void (*callback)(void *, Actuators::message*));

    void build_telemetry_messages();

public:


    ActuatorUnit(Actuators* command_bus, uint8_t id) {
        this->command_bus = command_bus;
        this->id = id;

        // Setup all the required messages for gathering information from the object
        this->reocurring_messages = new telemetry_message[10];
        this->build_telemetry_messages();
    }

    void update();

    /**
     * Queues all telemetry messages that need to be sent during this cycle
     */
    void queue_telemetry_messages();

    void set_control_mode(control_modes mode, uint8_t motor);

    void set_target_position(int32_t position_m1, int32_t position_m2);

    void emergency_stop();

    String* get_status_string();

};


#endif //TEENSYCANTRANSCEIVER_ACTUATORUNIT_H
