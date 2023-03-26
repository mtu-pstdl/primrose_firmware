//
// Created by Jay on 12/10/2022.
//

#ifndef TEENSYCANTRANSCEIVER_ODRIVES1_H
#define TEENSYCANTRANSCEIVER_ODRIVES1_H


#include <cstdint>
#include <cmath>
#include "odrive_constants.h"
#include "motor_configs.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Float32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"

#include <FlexCAN_T4.h>


class ODriveS1{

public:
    String* name = nullptr; // The name of the ODrive
    uint8_t can_id = 0;

private:
    FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64>* can_bus = nullptr; // The CAN bus pointer

    char* axis_error_string; // The axis error string
    char* axis_state_string; // The axis state string
    char* procedure_result_string; // The procedure result string
    char* active_errors_string; // The active errors string
    char* disarm_reason_string; // The disarm reason string

    void allocate_strings() {
        axis_error_string = new char[25];
        sprintf(axis_error_string, "Not initialized");
        axis_state_string = new char[25];
        sprintf(axis_state_string, "Not initialized");
        procedure_result_string = new char[25];
        sprintf(procedure_result_string, "Not initialized");
        active_errors_string = new char[25];
        sprintf(active_errors_string, "Not initialized");
        disarm_reason_string = new char[25];
        sprintf(disarm_reason_string, "Not initialized");
    }

    uint32_t last_message = 0; // The last time a message was received from the ODrive
    uint32_t in_flight_bitmask = 0; // The number of messages in flight

#define AXIS_STATE_UPDATE_RATE 100 // The rate at which the axis state is updated in ms
#define AXIS_STATE_FLIGHT_BIT 0x0001 // The bit in the in_flight_bitmask that corresponds to the axis state
    uint32_t   last_axis_state  = 0;   // The last axis state
    uint32_t   last_heartbeat   = 0;   // The time of the last heartbeat sent
    uint32_t   AXIS_ERROR       = 0;   // Axis error code
    uint8_t    AXIS_STATE       = 0;   // Axis state code
    uint8_t    PROCEDURE_RESULT = 0;   // Axis procedure result code

#define ERROR_UPDATE_RATE 100 // The rate at which the error state is updated in ms
#define ERROR_FLIGHT_BIT 0x0002 // The bit in the in_flight_bitmask that corresponds to the error state
    uint32_t   last_errors = 0; // The last motor state
    uint32_t   ACTIVE_ERRORS = 0;  // Active errors
    uint32_t   DISARM_REASON = 0;  // Disarm reason

#define ENCODER_UPDATE_RATE 100 // The rate at which the encoder state is updated in ms
#define ENCODER_FLIGHT_BIT 0x0004 // The bit in the in_flight_bitmask that corresponds to the encoder state
    uint32_t   last_encoder_state = 0; // The last encoder state
    float_t    POS_ESTIMATE = 0; // Encoder position in counts
    float_t    VEL_ESTIMATE = 0; // Encoder velocity in counts per second

#define IQ_UPDATE_RATE 100 // The rate at which the motor state is updated in ms
#define IQ_FLIGHT_BIT 0x0008 // The bit in the in_flight_bitmask that corresponds to the motor state
    uint32_t   last_iq_update = 0; // The last iq state
    float_t    Iq_Setpoint = 0;  // Iq setpoint in amps
    float_t    Iq_Measured = 0;  // Iq measured in amps

#define TEMP_UPDATE_RATE 100 // The rate at which the temperature state is updated in ms
#define TEMP_FLIGHT_BIT 0x0010 // The bit in the in_flight_bitmask that corresponds to the temperature state
    uint32_t   last_temp_update = 0; // The last temperature state
    float_t    FET_TEMP     = 0; // FET temperature in degrees Celsius
    float_t    MOTOR_TEMP   = 0; // Motor temperature in degrees Celsius

#define VBUS_UPDATE_RATE 100 // The rate at which the vbus state is updated in ms
#define VBUS_FLIGHT_BIT 0x0020 // The bit in the in_flight_bitmask that corresponds to the vbus state
    uint32_t   last_vbus_update = 0; // The last vbus state
    float_t    VBUS_VOLTAGE = 0; // Vbus voltage in volts
    float_t    VBUS_CURRENT = 0; // Vbus current in amps

#define HEARTBEAT_UPDATE_RATE 100 // The rate at which the heartbeat state is updated in ms
#define HEARTBEAT_FLIGHT_BIT 0x0040 // The bit in the in_flight_bitmask that corresponds to the heartbeat state

    float_t setpoint = 0; // The setpoint of the ODrive
    float_t velocity = 0; // The velocity of the ODrive
    float_t torque   = 0; // The torque of the ODrive

    uint32_t sent_messages = 0; // The number of messages sent to the ODrive

    bool conn_established = false; // Flag for if a connection has been established with the ODrive

    enum command_ids: uint8_t { // These are can bus command ids
        Heartbeat = 0x001, Estop = 0x002,
        Get_Error = 0x003, Set_Axis_Node_ID = 0x006,
        Set_Axis_State = 0x007, Get_Encoder_Estimates = 0x009,
        Set_Controller_Mode = 0x00b, Set_Input_Pos = 0x00c,
        Set_Input_Vel = 0x00d, Set_Input_Torque = 0x00e,
        Set_Limits = 0x00f, Set_Traj_Vel_Limit = 0x011,
        Set_Traj_Acc_Limit = 0x012, Set_Traj_Inertia = 0x013,
        Get_Iq = 0x014, Get_Temperature = 0x015,
        Reboot = 0x016, Get_Vbus_Voltage_Current = 0x017,
        Clear_Errors = 0x018, Set_Absolute_Position = 0x019,
        Set_Pos_Gains = 0x01a, Set_Vel_Gains = 0x01b,
        Get_ADC_Voltage = 0x01c, Get_Controller_Error = 0x01d,
    };

    uint8_t send_command(command_ids command_id);

    template <typename T>
    uint8_t send_command(command_ids command_id, T value);

    template <typename T>
    uint8_t send_command(command_ids command_id, T lower_data, T upper_data);

    void* estop_callback = nullptr; // The estop callback function

public:

    bool is_connected() const;

    ODriveS1(uint8_t can_id, String* name, FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64>* can_bus,
             void* estop_callback);

    void init(); // Initialize the ODrive module

    void on_message(const CAN_message_t &msg);

    void set_config(ODRIVE_MOTOR_CONFIG* config);

    void refresh_data(); // Refreshes data from the ODrive

    void estop(); // Sends an estop command to the ODrive


    // Getters and setters

//    String* get_state_string(); // Returns the state as a string

    uint8_t get_can_id() const;

    float_t get_fet_temp() const;

    float_t get_motor_temp() const;

    float_t get_vbus_voltage() const;

    float_t get_vbus_current() const;

    float_t get_pos_estimate() const;

    float_t get_vel_estimate() const;

    float_t get_Iq_setpoint() const;

    float_t get_Iq_measured() const;

    float_t get_setpoint() const;

    uint32_t get_axis_state() const;

    char* get_axis_state_string();

    uint32_t get_axis_error() const;

    char* get_axis_error_string();

    uint32_t get_active_errors() const;

    char* get_active_errors_string();

    uint32_t get_disarm_reason() const;

    char* get_disarm_reason_string();

};


#endif //TEENSYCANTRANSCEIVER_ODRIVES1_H
