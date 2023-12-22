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

class ODriveROS : public ROSNode {

    ODrivePro *odrive = nullptr;
    String name = ""; // The name of the ODrive

private:

    uint32_t last_ros_command = 0;

    enum ROS_COMMANDS {
        E_STOP = 0,            // 0 Arguments
        DISARM = 1,            // 0 Arguments
        CLEAR_ERRORS = 2,      // 0 Argument
        SET_CLOSED_LOOP = 3,   // 2 Argument  (control_mode, input_mode)
        SET_POINT = 4,         // 1 Argument  (setpoint)
        CALIBRATE = 5,         // 0 Arguments
    };

    ros::Subscriber<std_msgs::Int32MultiArray, ODriveROS> setpoint_sub;

    // Publishes the values of POS_ESTIMATE, VEL_ESTIMATE, IQ_SETPOINT, IQ_MEASURED
    std_msgs::Int32MultiArray* output_topic;

    // Publishes the values of AXIS_STATE, AXIS_ERROR, ACTIVE_ERRORS, DISARM_REASON
    diagnostic_msgs::DiagnosticStatus* state_topic;

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
        this->output_topic->data_length = 20;
        this->output_topic->data = new int32_t[20];
        this->output_topic->data[0]  = this->odrive->can_id;  // CAN ID (is static)
        this->output_topic->data[1]  = 0;  // POS_ESTIMATE
        this->output_topic->data[2]  = 0;  // VEL_ESTIMATE
        this->output_topic->data[3]  = 0;  // CURRENT_SETPOINT
        this->output_topic->data[4]  = 0;  // CONTROL_MODE
        this->output_topic->data[5]  = 0;  // AXIS_STATE
        this->output_topic->data[6]  = 0;  // AXIS_ERROR
        this->output_topic->data[7]  = 0;  // ACTIVE_ERRORS
        this->output_topic->data[8]  = 0;  // DISARM_REASON
        this->output_topic->data[9]  = 0;  // PROCEDURE_RESULT
        this->output_topic->data[10] = 0;  // TORQUE_ESTIMATE
        this->output_topic->data[11] = 0;  // VBUS_VOLTAGE
        this->output_topic->data[12] = 0;  // VBUS_CURRENT
        this->output_topic->data[13] = 0;  // ODOMETER_DISTANCE
        this->output_topic->data[14] = 0;  // ODOMETER_POWER
        this->output_topic->data[15] = 0;  // IQ_MEASURED
        this->output_topic->data[16] = 0;  // IQ_SETPOINT
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
