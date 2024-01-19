//
// Created by Jay on 3/16/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ACTUATORUNIT_H
#define TEENSYCANTRANSCEIVER_ACTUATORUNIT_H

#include <Arduino.h>
#include "Actuator_Bus_Interface.h"
#include "Misc/EStopDevice.h"
#include "Sensors_internal/PositionSensor.h"

#define ACTUATOR_SPEED 1000000
#define ACTUATOR_ACCEL 1000000
#define ACTUATOR_DECEL 1000000

#define M1_POS_MASK 0b00000001
#define M2_POS_MASK 0b00000010
#define CTTEMP_MASK 0b00000100
#define M2_VEL_MASK 0b00001000
#define CURENT_MASK 0b00010000
#define LG_BAT_MASK 0b00100000
#define MN_BAT_MASK 0b01000000
#define STATUS_MASK 0b10000000

extern Actuator_Bus_Interface ACTUATOR_BUS_INTERFACE;

class ActuatorUnit : public EStopDevice {

public:

    uint8_t  id;
    boolean  connected = true;
    uint32_t last_response = 0;

    enum fault_flags: uint16_t {
        ENCODER_INVALID = 0x0001,
        ENCODER_FAILURE = 0x0002,
        MAIN_BUS_LOW    = 0x0004,
        LOGIC_BUS_LOW   = 0x0008,
        MAIN_BUS_HIGH   = 0x0010,
        LOGIC_BUS_HIGH  = 0x0020,
        TEMP_HIGH       = 0x0040,
        CONNECTION_LOST = 0x0080,
        ESTOPPED        = 0x0100,
        CURRENT_LIMIT   = 0x0200,
    };

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
    } __attribute__((unused));

    enum control_modes {
        E_STOPPED  = 0,
        DUTY_CYCLE = 1,
        POSITION = 2,
        VELOCITY = 3,
    };

    struct odometer_value {  // stores odometer data in RAM
        bool     changed;     // Indicates whether odometer data has changed since last save
        uint32_t sequence_id; // Sequence ID (used to determine which odometer data is most recent)
        uint32_t odometer;    // Unit: 100th of a turn or encoder counts (depending on encoder type)
        uint32_t used_power;  // Unit: milliwatt-hours
    };

    struct motor_data {

        // Working variables
        int32_t target_position     = 0;         // The target position of the motor in analog value
        PositionSensor* encoder     = nullptr;   // The encoder of the motor
        float_t i_term              = 0;         // The integral term of the motor
        float_t duty_cycle_limit    = 0;         // The maximum duty cycle of the motor based the current power draw
        float_t current_duty_cycle  = 0;         // The current duty cycle of the motor
        int16_t current_current     = INT16_MIN; // The current current draw of the motor in ma
        control_modes control_mode  = E_STOPPED; // The current control mode of the motor
        boolean achieved_position   = false;     // Whether the motor has achieved its target position
        odometer_value* odometer    = nullptr;   // The odometer data of the motor

        // Fault flags
        uint16_t fault_flags        = 0;         // A bitmask of the faults that have occurred on the motor

        // Configuration variables
        float_t p_gain               = -0.01;     // The proportional gain of the motor
        float_t i_gain               = -0.001;    // The integral gain of the motor
        float_t max_duty_cycle       = 0.5;       // The maximum duty cycle of the motor
        int32_t position_tolerance   = 2;         // The distance from the target position that is considered on target
        int32_t activation_tolerance = 10;        // The distance from the target position that is considered off target
        int16_t current_limit        = 200;       // The current current draw of the motor in ma
        int32_t max_extension        = 0;         // The maximum extension of the motor in analog value
        boolean reversed             = false;     // Whether the motor is reversed
        boolean has_limit            = false;     // Whether the motor has a limit
        boolean limit_direction      = false;     // Which direction the limit should apply from
        boolean limit_action_dir     = false;     // Which direction the limit action should apply from
    };

    static void motor_currents_callback(void *actuator, Actuator_Bus_Interface::serial_message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        actuator_unit->last_response = millis();
        actuator_unit->motors[0].current_current =
                static_cast<int16_t>((msg->data[0] << 8) | msg->data[1]);
        // Because the current sensors don't seem to zero properly, we ignore anything below less than 0
        if (actuator_unit->motors[0].current_current < 0) actuator_unit->motors[0].current_current = 0;
        actuator_unit->motors[1].current_current =
                static_cast<int16_t>((msg->data[2] << 8) | msg->data[3]);
        if (actuator_unit->motors[1].current_current < 0) actuator_unit->motors[1].current_current = 0;
        actuator_unit->current_limit_check(0);
        actuator_unit->current_limit_check(1);
        actuator_unit->data_flags |= CURENT_MASK;
    }

    static void main_battery_voltage_callback(void *actuator, Actuator_Bus_Interface::serial_message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        actuator_unit->last_response = millis();
        actuator_unit->main_battery_voltage = (msg->data[0] << 8) | msg->data[1];
        actuator_unit->data_flags |= MN_BAT_MASK;
    }

    static void logic_battery_voltage_callback(void *actuator, Actuator_Bus_Interface::serial_message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        actuator_unit->last_response = millis();
        actuator_unit->logic_battery_voltage = (msg->data[0] << 8) | msg->data[1];
        actuator_unit->data_flags |= LG_BAT_MASK;
    }

    static void controller_temp_callback(void *actuator, Actuator_Bus_Interface::serial_message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->data_flags |= CTTEMP_MASK;
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        actuator_unit->last_response = millis();
        actuator_unit->controller_temperature = (msg->data[0] << 8) | msg->data[1];
    }

    static void controller_status_callback(void *actuator, Actuator_Bus_Interface::serial_message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        actuator_unit->message_dropped_count = 0;
        actuator_unit->connected = true;
        actuator_unit->last_response = millis();
        actuator_unit->status = msg->data[0];
//        actuator_unit->status = (msg->data[0] << 8) | msg->data[1];
        actuator_unit->data_flags |= STATUS_MASK;
    }

    static void command_message_callback(void *actuator, Actuator_Bus_Interface::serial_message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
//        actuator_unit->message_dropped_count = 0;
//        actuator_unit->connected = true;
    }

    static void command_failure_callback(void *actuator, Actuator_Bus_Interface::serial_message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        // Because command messages are more important than status messages we must resend them if they fail
//        if (actuator_unit->command_bus->space_available())
//            actuator_unit->command_bus->queue_message(msg);

        actuator_unit->message_dropped_count++;
        if (actuator_unit->message_dropped_count > actuator_unit->message_failure_threshold) {
            actuator_unit->connected = false;
        }
    }

    static void message_failure_callback(void *actuator, Actuator_Bus_Interface::serial_message *msg) {
        auto* actuator_unit = static_cast<ActuatorUnit*>(actuator);
        // Check if the serial_message failed because of CRC failure
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

    Actuator_Bus_Interface* command_bus;

    uint16_t message_dropped_count = 0;
    uint16_t message_failure_count = 0;
    const uint16_t message_failure_threshold = 20;

    motor_data motors[2]{
            motor_data(),
            motor_data()
    };

    struct telemetry_message{
        Actuator_Bus_Interface::serial_message* msg; // The serial_message to send
        uint32_t last_send_time; // The time the serial_message was last sent
        uint32_t send_interval;  // The minimum time between sending the serial_message
    };

    telemetry_message reoccurring_messages[7];
    telemetry_message command_messages[2];

    // Shared variables between motor 1 and motor 2
    uint16_t controller_temperature = UINT16_MAX;
    uint16_t main_battery_voltage   = UINT16_MAX;
    uint16_t logic_battery_voltage  = UINT16_MAX;
    uint16_t status = 0;
    uint16_t data_flags = 0;  // A bitmask of the data that has been received from the motor

    telemetry_message* build_message(Actuator_Bus_Interface::serial_commands command, uint32_t send_interval,
                                     uint8_t data_length,
                                     void (*callback)(void *, Actuator_Bus_Interface::serial_message*));

    void build_telemetry_messages();

    void update_duty_cycle_command(float_t duty_cycle, uint8_t motor, boolean send_immediately = false);

public:


    ActuatorUnit(uint8_t id, PositionSensor* steering_encoder, PositionSensor* suspension_encoder) {
        this->id = id;
        this->command_bus = &ACTUATOR_BUS_INTERFACE;

        this->motors[0].encoder = suspension_encoder;
        this->motors[1].encoder = steering_encoder;

        // Setup all the required messages for gathering information from the object
        this->command_messages[0].msg = new Actuator_Bus_Interface::serial_message();
        this->command_messages[1].msg = new Actuator_Bus_Interface::serial_message();
        for (auto & command_message : this->command_messages) {
            command_message.send_interval = 100;
            command_message.msg->id = this->id;
            command_message.msg->object = this;
            command_message.msg->callback = &command_message_callback;
            command_message.msg->failure_callback = &command_failure_callback;
            command_message.msg->expect_response = false;
            command_message.msg->protected_action = true;
            command_message.msg->free_after_callback = false;
        }
        this->set_duty_cycle(0, 0);
        this->set_duty_cycle(0, 1);

        this->build_telemetry_messages();
    }

    void update();

    void set_inverted(bool inverted, uint8_t motor);

    void set_limits(uint8_t motor, int32_t limit, boolean direction, boolean action_direction);

    void estop() override;

    void resume() override;

    EStopDevice::TRIP_LEVEL tripped(char* device_name, char* device_message) override;

    void pass_odometer(odometer_value* odometer, uint8_t motor);

    /**
     * Queues all telemetry messages that need to be sent during this cycle
     */
    void queue_telemetry_messages();

    void set_target_position(int32_t position, uint8_t motor);

    void set_duty_cycle(float_t duty_cycle, uint8_t motor);

    int32_t get_target_position(uint8_t motor);

    control_modes get_control_mode(uint8_t motor);

    uint16_t get_status() const;

    int32_t get_position(uint8_t motor);

    float_t get_velocity(uint8_t motor);

    float_t get_duty_cycle(uint8_t motor);

    float_t get_current(uint8_t motor);

    float_t get_temperature() const;

    float_t get_main_battery_voltage() const;

    float_t get_logic_battery_voltage() const;

    uint16_t get_fault_flags(uint8_t motor);

    void update_odometer(uint8_t motor);

    void update_power_consumption(uint8_t motor);

    void current_limit_check(uint8_t motor);

    void check_connection();

    void position_control_callback(uint8_t motor);

    boolean on_target(uint8_t motor);
};


#endif //TEENSYCANTRANSCEIVER_ACTUATORUNIT_H
