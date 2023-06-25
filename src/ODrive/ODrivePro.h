//
// Created by Jay on 12/10/2022.
//

#ifndef TEENSYCANTRANSCEIVER_ODRIVEPRO_H
#define TEENSYCANTRANSCEIVER_ODRIVEPRO_H


#include <cstdint>
#include <cmath>
#include "odrive_constants.h"
#include "motor_configs.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Float32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"

#include <FlexCAN_T4.h>


class ODrivePro{

public:
    String* name = nullptr; // The name of the ODrive
    uint8_t can_id = 0;

    char* vel_unit_string; // The unit string
    char* pos_unit_string; // The unit string

    struct memory_odometer_value {  // stores odometer data in RAM
        bool     changed;     // Indicates whether odometer data has changed since last save
        uint32_t sequence_id; // Sequence ID (used to determine which odometer data is most recent)
        uint32_t odometer;    // Unit: 10th of a turn
        uint32_t used_power;  // Unit: watt-hours (scaled by 10)
    };

private:

    bool    calibrating = false; // Whether or not the ODrive is calibrating
    uint8_t calibration_step = 0; // The current calibration step

    bool has_feedforward = false; // Whether or not the ODrive has feedforward
    feedforward_struct* feedforward = nullptr; // The feedforward value

    FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64>* can_bus = nullptr; // The CAN bus pointer

    char* axis_error_string; // The axis error string
    char* axis_state_string; // The axis state string
    char* procedure_result_string; // The procedure result string
    char* active_errors_string; // The active errors string
    char* disarm_reason_string; // The disarm reason string
    char* control_mode_string; // The control mode string
    char* input_mode_string; // The input mode string
    char* setpoint_string; // The setpoint string

    void allocate_strings() {
        axis_error_string = new char[50];
        sprintf(axis_error_string, "Not initialized");
        axis_state_string = new char[50];
        sprintf(axis_state_string, "Not initialized");
        procedure_result_string = new char[50];
        sprintf(procedure_result_string, "Not initialized");
        active_errors_string = new char[50];
        sprintf(active_errors_string, "Not initialized");
        disarm_reason_string = new char[50];
        sprintf(disarm_reason_string, "Not initialized");
        control_mode_string = new char[25];
        sprintf(control_mode_string, "Not initialized");
        setpoint_string = new char[25];
        sprintf(setpoint_string, "Not initialized");
        input_mode_string = new char[25];
        sprintf(input_mode_string, "Not initialized");
        vel_unit_string = new char[5];
        sprintf(vel_unit_string, "RPS");
        pos_unit_string = new char[5];
        sprintf(pos_unit_string, "Revs");
    }

    uint32_t last_message = 0; // The last time a message was received from the ODrive
    uint32_t in_flight_bitmask = 0; // The number of messages in flight

#define AXIS_STATE_UPDATE_RATE 110 // The rate at which the axis state is updated in ms
#define AXIS_STATE_FLIGHT_BIT 0x0001 // The bit in the in_flight_bitmask that corresponds to the axis state
    uint32_t   last_axis_state  = 0;   // The last axis state time
    uint32_t   last_heartbeat   = 0;   // The time of the last heartbeat sent

    uint32_t                     AXIS_ERROR       = INITIALIZING;   // Axis error code
    odrive::axis_states          AXIS_STATE       = odrive::UNDEFINED;
    odrive::procedure_results    PROCEDURE_RESULT = odrive::UNKNOWN_PROCEDURE_RESULT;

#define ERROR_UPDATE_RATE 110 // The rate at which the error state is updated in ms
#define ERROR_FLIGHT_BIT 0x0002 // The bit in the in_flight_bitmask that corresponds to the error state
    uint32_t   last_errors_update = 0; // The last motor state

    uint32_t   ACTIVE_ERRORS = 0;  // Active errors
    uint32_t   DISARM_REASON = 0;  // Disarm reason

#define ENCODER_UPDATE_RATE 110 // The rate at which the encoder state is updated in ms
#define ENCODER_FLIGHT_BIT 0x0004 // The bit in the in_flight_bitmask that corresponds to the encoder state
    uint32_t   last_encoder_update = 0; // The last encoder state

    float_t    POS_ESTIMATE = 0; // Encoder position in counts
    float_t    VEL_ESTIMATE = 0; // Encoder velocity in counts per second

#define IQ_UPDATE_RATE 110 // The rate at which the motor state is updated in ms
#define IQ_FLIGHT_BIT 0x0008 // The bit in the in_flight_bitmask that corresponds to the motor state
    uint32_t   last_iq_update = 0; // The last iq state

    float_t    IQ_SETPOINT = 0;  // Iq setpoint in amps
    float_t    IQ_MEASURED = 0;  // Iq measured in amps

#define TEMP_UPDATE_RATE 110 // The rate at which the temperature state is updated in ms
#define TEMP_FLIGHT_BIT 0x0010 // The bit in the in_flight_bitmask that corresponds to the temperature state
    uint32_t   last_temp_update = 0; // The last temperature state

    float_t    FET_TEMP     = 0; // FET temperature in degrees Celsius
    float_t    MOTOR_TEMP   = 0; // Motor temperature in degrees Celsius

#define VBUS_UPDATE_RATE 110 // The rate at which the vbus state is updated in ms
#define VBUS_FLIGHT_BIT 0x0020 // The bit in the in_flight_bitmask that corresponds to the vbus state
    uint32_t   last_vbus_update = 0; // The last vbus state

    float_t    VBUS_VOLTAGE = 0; // Vbus voltage in volts
    float_t    VBUS_CURRENT = 0; // Vbus current in amps

#define TORQUE_UPDATE_RATE 110 // The rate at which the torque state is updated in ms
#define TORQUE_FLIGHT_BIT 0x0040 // The bit in the in_flight_bitmask that corresponds to the torque state
    uint32_t   last_torque_update = 0; // The last torque state

    float_t    TORQUE_TARGET = 0; // Torque setpoint in Nm
    float_t    TORQUE_ESTIMATE = 0; // Torque measured in Nm

#define HEARTBEAT_UPDATE_RATE 110 // The rate at which the heartbeat state is updated in ms
#define HEARTBEAT_FLIGHT_BIT 0x0040 // The bit in the in_flight_bitmask that corresponds to the heartbeat state

    float_t position_setpoint = 0; // The position of the ODrive
    float_t velocity_setpoint = 0; // The velocity of the ODrive
    float_t torque_setpoint   = 0; // The torque of the ODrive

    odrive::control_modes control_mode = odrive::UNKNOWN_CONTROL_MODE; // The control mode of the ODrive

    odrive::input_modes input_mode = odrive::UNKNOWN_INPUT_MODE; // The input mode of the ODrive

    bool has_rev_conversion = false; // Whether or not the ODrive has a conversion from ticks to revolutions
    float_t ticks_per_rev = 0;  // The number of ticks per revolution of output shaft
    bool has_meter_conversion = false; // Whether or not the ODrive has a conversion from revs to meters
    float_t meter_per_rev = 0; // The number of revolutions of the output shaft per meter of distance

    float_t last_pos = 0; // The last position of the ODrive

    memory_odometer_value* odometer = nullptr; // The odometer to update

    uint8_t send_command(odrive::command_ids command_id);

    template <typename T>
    uint8_t send_command(odrive::command_ids command_id, T value);

    template <typename T1, typename T2>
    uint8_t send_command(odrive::command_ids command_id, T1 lower_data, T2 upper_data);

    void* estop_callback = nullptr; // The emergency_stop callback function

    void calibration_sequence(); // The calibration sequence for the ODrive

    void update_power_consumption(float_t voltage, float_t current); // Updates the power consumption of the ODrive

    void update_odometer(); // Updates the odometer of the ODrive

public:

    bool is_connected() const;

    ODrivePro(uint8_t can_id, FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64>* can_bus,
              void* estop_callback);

    void pass_odometer_data(void* pointer);

    void set_feedforward(feedforward_struct* feedforward);

    void init();

    void reboot(); // Reboots the ODrive

    void on_message(const CAN_message_t &msg);

    void set_control_mode(odrive::control_modes mode, odrive::input_modes input_mode);

    void calibrate();

    void set_setpoint(float_t value);

//    void set_limits(float_t vel_limit);

    void refresh_data(); // Refreshes data from the ODrive

    void emergency_stop(); // Sends an emergency_stop command to the ODrive


    // Getters and setters

    uint8_t get_can_id() const;

    float_t get_fet_temp() const;

    float_t get_motor_temp() const;

    float_t get_vbus_voltage() const;

    float_t get_vbus_current() const;

    float_t get_torque_target() const;

    float_t get_torque_estimate() const;

    float_t get_pos_estimate() const;

    float_t get_vel_estimate() const;

    float_t get_Iq_setpoint() const;

    float_t get_Iq_measured() const;

    double_t get_odometer() const;

    void set_axis_state(odrive::axis_states state);

    odrive::axis_states get_axis_state() const;

    char* get_axis_state_string();

    uint32_t get_axis_error() const;

    char* get_axis_error_string();

    uint32_t get_active_errors() const;

    char* get_active_errors_string();

    uint32_t get_disarm_reason() const;

    char* get_disarm_reason_string();

    odrive::procedure_results get_procedure_results() const;

    char* get_procedure_results_string();

    odrive::control_modes get_control_mode() const;

    odrive::input_modes get_input_mode() const;

    char* get_control_mode_string();

    char* get_input_mode_string();

    float_t get_setpoint() const;

    char* get_setpoint_string();

    bool has_error() const;

    uint32_t get_last_update() const;

    float_t unit_conversion(float_t value, bool direction) const;

    uint32_t get_inflight_bitmask() const;

    void clear_errors();

    float_t calculate_feedforward(float_t setpoint);
};


#endif //TEENSYCANTRANSCEIVER_ODRIVEPRO_H
