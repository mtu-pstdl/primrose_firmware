//
// Created by Jay on 12/10/2022.
//

#ifndef TEENSYCANTRANSCEIVER_ODRIVES1_H
#define TEENSYCANTRANSCEIVER_ODRIVES1_H

#include <cstdint>

#include <FlexCAN_T4.h>
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"


class ODriveS1{

    String name = "";
    uint8_t can_id = 0;

    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64>* can_bus = nullptr; // The CAN bus pointer

#define AXIS_REFRESH_BIT 0x01
    float_t    AXIS_ERROR = 0;   // Axis error code
    uint32_t   AXIS_STATE = 0;   // Axis state code
#define ERROR_REFRESH_BIT 0x02
    uint32_t   ACTIVE_ERRORS = 0;  // Active errors
    uint32_t   DISARM_REASON = 0;  // Disarm reason
#define ENCODER_REFRESH_BIT 0x04
    float_t    POS_ESTIMATE = 0; // Encoder position in counts
    float_t    VEL_ESTIMATE = 0; // Encoder velocity in counts per second
#define IQ_REFRESH_BIT 0x08
    float_t    Iq_Setpoint = 0;  // Iq setpoint in amps
    float_t    Iq_Measured = 0;  // Iq measured in amps
#define TEMP_REFRESH_BIT 0x10
    float_t    FET_TEMP     = 0; // FET temperature in degrees Celsius
    float_t    MOTOR_TEMP   = 0; // Motor temperature in degrees Celsius
#define VBUS_REFRESH_BIT 0x20
    float_t    VBUS_VOLTAGE = 0; // Vbus voltage in volts
    float_t    VBUS_CURRENT = 0; // Vbus current in amps

    uint8_t refresh_flags = 0; // Flags for which data has been refreshed to indicate if the refresh was successful
#define ODRIVE_REFRESH_FLAG_MASK 0x3F // Mask for the refresh flags
    uint32_t last_refresh = 0; // The last time the data completely refreshed
    uint32_t last_refresh_attempt = 0; // The last time the data was attempted to be refreshed


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
    };

    uint8_t send_command(command_ids command_id);

    uint8_t send_command(command_ids command_id, uint32_t lower_data, uint32_t upper_data);

public:

    void init(ros::NodeHandle *node_ptr);

    ODriveS1(uint8_t can_id, String name, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64>* can_bus);

    void on_message(const CAN_message_t &msg);

    void refresh_data(); // Refreshes data from the ODrive

    String* get_state_string(); // Returns the state as a string

    uint8_t get_can_id() const;
};


#endif //TEENSYCANTRANSCEIVER_ODRIVES1_H
