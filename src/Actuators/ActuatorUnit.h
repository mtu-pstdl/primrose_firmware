//
// Created by Jay on 3/16/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ACTUATORUNIT_H
#define TEENSYCANTRANSCEIVER_ACTUATORUNIT_H

#include <Arduino.h>
#include "Actuators.h"
#define ACTUATOR_SPEED 1000000
#define ACTUATOR_ACCEL 1000000
#define ACTUATOR_DECEL 1000000

#define M1_POS_MASK 0b00000001
#define M2_POS_MASK 0b00000010
#define M1_VEL_MASK 0b00000100
#define M2_VEL_MASK 0b00001000
#define CURENT_MASK 0b00010000
#define LG_BAT_MASK 0b00100000
#define MN_BAT_MASK 0b01000000
#define STATUS_MASK 0b10000000

class ActuatorUnit {

public:

    uint8_t id;
    boolean connected = true;

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
        m2_homing                = 0x2000
    };

    enum control_modes {
        stopped  = 0,
        position = 1,
        velocity = 2,
        homing   = 3
    };

    struct motor_info{
        char*    name                = nullptr; // The name of the motor
        int32_t target_position     = 0; // The target position of the motor ticks
        int32_t current_position    = 0; // The current position of the motor in ticks
        int32_t max_position        = 0; // The maximum position of the motor in ticks
        int32_t current_speed       = 0; // The current velocity of the motor in ticks per second
        int16_t current_current     = 7; // The current current draw of the motor in ma
        int16_t warning_current     = 50; // The current current draw of the motor in ma
        control_modes control_mode   = stopped; // The current control mode of the motor
        boolean  homed               = true; // Whether or not the motor has been homed
        boolean  fault               = false; // Whether or not the motor has a fault
        boolean  warn                = false; // Whether or not the motor has a warning
        char*    status_string       = nullptr; // A string describing the diagnostics_topic of the motor
    };

    static void detailed_encoder_count_callback(void* actuator, Actuators::message* msg);

    // Welcome to pointer hell

    static void encoder_count_callback(void *actuator, Actuators::message *msg) {
        // Cast the void pointer to an ActuatorUnit pointer
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        uint32_t raw_position = (msg->data[0] << 24) | (msg->data[1] << 16) | (msg->data[2] << 8) | msg->data[3];
        bool negative = msg->data[4] & 0b00000010;
        if (msg->command == Actuators::serial_commands::read_encoder_count_m1){
            // Trim the position value to a 32 bit signed integer
            if (negative) {
                actuator_unit->motors[0].current_position = - (int32_t) raw_position;
            } else actuator_unit->motors[0].current_position = (int32_t) raw_position;
            actuator_unit->data_flags |= M1_POS_MASK;
        } else if (msg->command == Actuators::serial_commands::read_encoder_count_m2){
            if (negative) {
                actuator_unit->motors[1].current_position = - (int32_t) raw_position;
            } else actuator_unit->motors[1].current_position = (int32_t) raw_position;
            actuator_unit->data_flags |= M2_POS_MASK;
        }
    }

    static void encoder_speed_callback(void *actuator, Actuators::message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        if (msg->command == Actuators::serial_commands::read_encoder_speed_m1){
            uint32_t raw_speed = (msg->data[0] << 24) | (msg->data[1] << 16) | (msg->data[2] << 8) | msg->data[3];
            if (msg->data[4]) {
                actuator_unit->motors[0].current_speed = - (int32_t) raw_speed;
            } else actuator_unit->motors[0].current_speed = (int32_t) raw_speed;
            actuator_unit->data_flags |= M1_VEL_MASK;
        } else if (msg->command == Actuators::serial_commands::read_encoder_speed_m2){
            uint32_t raw_speed = (msg->data[0] << 24) | (msg->data[1] << 16) | (msg->data[2] << 8) | msg->data[3];
            if (msg->data[4]) {
                actuator_unit->motors[1].current_speed = - (int32_t) raw_speed;
            } else actuator_unit->motors[1].current_speed = (int32_t) raw_speed;
            actuator_unit->data_flags |= M2_VEL_MASK;
        }
    }

    static void motor_currents_callback(void *actuator, Actuators::message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        actuator_unit->motors[0].current_current =
                static_cast<int16_t>((msg->data[0] << 8) | msg->data[1]);
        actuator_unit->motors[1].current_current =
                static_cast<int16_t>((msg->data[2] << 8) | msg->data[3]);
        actuator_unit->data_flags |= CURENT_MASK;
    }

    static void main_battery_voltage_callback(void *actuator, Actuators::message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        actuator_unit->main_battery_voltage = (msg->data[0] << 8) | msg->data[1];
        actuator_unit->data_flags |= MN_BAT_MASK;
    }

    static void logic_battery_voltage_callback(void *actuator, Actuators::message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        actuator_unit->logic_battery_voltage = (msg->data[0] << 8) | msg->data[1];
        actuator_unit->data_flags |= LG_BAT_MASK;
    }

    static void controller_temp_callback(void *actuator, Actuators::message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        actuator_unit->controller_temperature = (msg->data[0] << 8) | msg->data[1];
    }

    static void controller_status_callback(void *actuator, Actuators::message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        actuator_unit->status = (msg->data[0] << 8) | msg->data[1];
        actuator_unit->data_flags |= STATUS_MASK;
    }

    static void command_message_callback(void *actuator, Actuators::message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
    }

    static void command_failure_callback

    static void message_failure_callback(void *actuator, Actuators::message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        // Check if the message failed because of CRC failure
        // (This indicates the controller is connected but the connection is noisy and a separate failure)
        if (msg->failed_crc){
            actuator_unit->message_failure_count++;
        }

        actuator_unit->message_dropped_count++;
        if (actuator_unit->message_dropped_count > actuator_unit->message_failure_threshold) {
            actuator_unit->connected = false;
        }
    }

private:

    Actuators* command_bus;

    uint16_t message_dropped_count = 0;
    uint16_t message_failure_count = 0;
    const uint16_t message_failure_threshold = 10;

    char* status_string = nullptr;

    void allocate_strings(){
        this->status_string = new char[100];
        this->motors[0].name = new char[5];
        this->motors[1].name = new char[5];
        sprintf(this->motors[0].name, "M1");
        sprintf(this->motors[1].name, "M2");
        this->motors[0].status_string = new char[256];
        this->motors[1].status_string = new char[256];
    }

    motor_info motors[2]{
            motor_info(),
            motor_info()
    };

    struct telemetry_message{
        Actuators::message* msg; // The message to send
        uint32_t last_send_time; // The time the message was last sent
        uint32_t send_interval;  // The minimum time between sending the message
    };

    struct command_message{
        Actuators::message* msg; // The message to send
        uint32_t last_send_time; // The time the message was last sent
        uint32_t send_interval;  // The minimum time between sending the message
    };

    telemetry_message* reocurring_messages;
    command_message* command_messages;

    // Shared variables between motor 1 and motor 2
    uint16_t controller_temperature = 0;
    uint16_t main_battery_voltage   = 0;
    uint16_t logic_battery_voltage  = 0;
    uint16_t status = 0;
    uint8_t  data_flags = 0;  // A bitmask of the data that has been received from the motor

    telemetry_message* build_message(Actuators::serial_commands command, uint32_t send_interval, uint8_t data_length,
                                     void (*callback)(void *, Actuators::message*));

    void build_telemetry_messages();

    void send_target_position(uint8_t motor);

public:


    ActuatorUnit(Actuators* command_bus, uint8_t id) {
        this->command_bus = command_bus;
        this->id = id;

        // Setup all the required messages for gathering information from the object
        this->reocurring_messages = new telemetry_message[7];
        this->command_messages = new command_message[2];
        this->command_messages[0].msg = new Actuators::message();
        this->command_messages[1].msg = new Actuators::message();
        this->command_messages[0].msg->id = this->id;
        this->command_messages[1].msg->id = this->id;
        this->command_messages[0].msg->free_after_callback = false;
        this->command_messages[1].msg->free_after_callback = false;
        this->command_messages[0].send_interval = 1000;
        this->command_messages[1].send_interval = 1000;
        this->command_messages[0].msg->object = this;
        this->command_messages[1].msg->object = this;
        this->command_messages[0].msg->callback = &command_message_callback;
        this->command_messages[1].msg->callback = &command_message_callback;
        this->command_messages[0].msg->failure_callback = &message_failure_callback;
        this->command_messages[1].msg->failure_callback = &message_failure_callback;
        this->command_messages[0].msg->expect_response = false;
        this->command_messages[1].msg->expect_response = false;

        this->set_duty_cycle(0, 0);
        this->set_duty_cycle(0, 1);

        this->build_telemetry_messages();
        this->allocate_strings();
    }

    void update();

    void estop();

    /**
     * Queues all telemetry messages that need to be sent during this cycle
     */
    void queue_telemetry_messages();

    void set_control_mode(control_modes mode, uint8_t motor);

    void set_target_position(int32_t position, uint8_t motor);

    int32_t get_target_position(uint8_t motor);

    void emergency_stop();

    uint16_t get_status() const;

    char* get_motor_fault_string(uint8_t motor);

    char* get_status_string();

    int32_t get_position(uint8_t motor);

    int32_t get_velocity(uint8_t motor);

    float_t get_current(uint8_t motor);

    float_t get_temperature() const;

    float_t get_main_battery_voltage() const;

    float_t get_logic_battery_voltage() const;

    void check_connection();

    void set_duty_cycle(float_t duty_cycle, uint8_t motor);
};


#endif //TEENSYCANTRANSCEIVER_ACTUATORUNIT_H
