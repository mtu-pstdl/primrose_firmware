//
// Created by Jay on 12/20/2022.
//

#ifndef TEENSYCANTRANSCEIVER_ODRIVE_ROS_H
#define TEENSYCANTRANSCEIVER_ODRIVE_ROS_H


#include "odrive_constants.h"
#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/node_handle.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/publisher.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/service_server.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Float32MultiArray.h"
#include "ODrivePro.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/UInt32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Empty.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/KeyValue.h"

#define UNIT_SCALE 100
//#define VEL_UNIT_SCALE 10000

/**
 * The ODriveROS class takes data from the ODrive and publishes it to the ROS network. It also takes
 * data from the ROS network and sends it to the ODrive.
 */
class ODriveROS : public ROSNode {

    ODrivePro *odrive = nullptr;
    String name = ""; // The name of the ODrive

private:

    /**
     * A named union for storing the output data into the Int32MultiArray message
     */
    union OutputArray {
        struct OutputData {
            int32_t can_id = 0;            // The CAN ID of the ODrive (static)
            int32_t pos_estimate = 0;      // Fixed point, x100 (unit: turns)
            int32_t vel_estimate = 0;      // Fixed point, x100 (unit: turns/s)
            int32_t current_setpoint = 0;  // Fixed point, x100 (unit: ControlMode dependent)
            int32_t control_mode = 0;      // odrive_constants::control_modes
            int32_t axis_state = 0;        // odrive_constants::axis_states
            int32_t axis_error = 0;        // AXIS_ERRORS bitmask
            int32_t active_errors = 0;     // AXIS_ERRORS bitmask
            int32_t disarm_reason = 0;     // AXIS_ERRORS bitmask
            int32_t procedure_result = 0;  // odrive_constants::procedure_results
            int32_t torque_estimate = 0;   // Fixed point, x100 (unit: Nm)
            int32_t vbus_voltage = 0;      // Fixed point, x100 (unit: V)
            int32_t vbus_current = 0;      // Fixed point, x100 (unit: A)
            int32_t odometer_distance = 0; // Fixed point, x100 (unit: kTurns)
            int32_t odometer_power = 0;    // Fixed point, x100 (unit: kW)
            int32_t iq_measured = 0;       // Fixed point, x100 (unit: A)
            int32_t iq_setpoint = 0;       // Fixed point, x100 (unit: A)
            int32_t fet_temperature = 0;   // Fixed point, x100 (unit: degC)
            int32_t mtr_temperature = 0;   // Fixed point, x100 (unit: degC)
            int32_t reserved[1] = {0};     // Reserved for future use
        } data;
        int32_t raw_array[20]{};  // The raw array of data to be sent over the serial bus
    } output_data = {};

    enum ROS_COMMANDS : int32_t {
        E_STOP = 0,            // 0 Arguments
        DISARM = 1,            // 0 Arguments
        CLEAR_ERRORS = 2,      // 0 Argument
        SET_CLOSED_LOOP = 3,   // 2 Arguments  (control_mode, input_mode)
        SET_POINT = 4,         // 1 Argument   (setpoint)
        CALIBRATE = 5,         // 0 Arguments
    };

    /**
     * A named union for storing the input data from the Int32MultiArray message
     * The start of the array is the ROS_COMMANDS enum which determines the layout of the rest of the array
     */
    union InputArray {
        struct InputData {
            ROS_COMMANDS command;
            union Arguments {  // Contains the possible data layouts for each command type
                struct SetClosedLoop {
                    int32_t control_mode;
                    int32_t input_mode;
                } set_closed_loop;
                int32_t set_point;
            } args;
        } data;
        int32_t raw_array[sizeof (InputData) / sizeof (int32_t)];
    } input_data = {};

    uint32_t last_ros_command = 0;

    ros::Subscriber<std_msgs::Int32MultiArray, ODriveROS> setpoint_sub;

    // Publishes the values of POS_ESTIMATE, VEL_ESTIMATE, IQ_SETPOINT, IQ_MEASURED
    std_msgs::Int32MultiArray* output_topic;

#define NUM_CONDITIONS 13
    char* strings[NUM_CONDITIONS]{};
    char* status_string = new char[25];
    char* setpoint_topic_name = new char[25];

public:


    ODriveROS(ODrivePro* odrive,
               std_msgs::Int32MultiArray* encoder_topic,
               String disp_name) :
            setpoint_sub("template1", &ODriveROS::setpoint_callback, this) {
        this->odrive = odrive;
        this->output_topic = encoder_topic;
        this->output_topic->data_length = sizeof (this->output_data.raw_array) / sizeof (int32_t);
        this->output_topic->data = this->output_data.raw_array;
        this->name = disp_name.c_str();
        this->setpoint_sub.topic_ = setpoint_topic_name;
        sprintf(setpoint_topic_name, "/mciu/%s/odrive/input", disp_name.c_str());
    }


    ODrivePro* get_odrive();

    void setpoint_callback(const std_msgs::Int32MultiArray &msg);

    void subscribe(ros::NodeHandle* node_handle) override;

    void update() override;

    static int32_t to_fixed_point(float value, float scale);

    static float from_fixed_point(int32_t value, float scale);

    static int32_t to_fixed_point(double_t value, float scale);
};


#endif //TEENSYCANTRANSCEIVER_ODRIVE_ROS_H
