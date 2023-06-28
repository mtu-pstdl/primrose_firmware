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

#define POS_UNIT_SCALE 100
//#define VEL_UNIT_SCALE 10000

class ODrive_ROS : public ROSNode {

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
        SET_VEL_LIMIT = 6,     // 1 Argument
    };

    ros::Subscriber<std_msgs::Int32MultiArray, ODrive_ROS> setpoint_sub;

    // Publishes the values of POS_ESTIMATE, VEL_ESTIMATE, IQ_SETPOINT, IQ_MEASURED
    std_msgs::Int32MultiArray* output_topic;

    // Publishes the values of AXIS_STATE, AXIS_ERROR, ACTIVE_ERRORS, DISARM_REASON
    diagnostic_msgs::DiagnosticStatus* state_topic;

#define NUM_CONDITIONS 13
    char* strings[NUM_CONDITIONS]{};
    char* status_string = new char[25];
    char* setpoint_topic_name = new char[25];

    void update_diagnostics_keys(bool error_mode){
        if (error_mode){
            state_topic->values[1].key = "AXIS_ERROR";        // Or CONTROL_MODE
            state_topic->values[2].key = "ACTIVE_ERRORS";     // Or INPUT_MODE
            state_topic->values[3].key = "DISARM_REASON";     // Or SETPOINT
            state_topic->values[4].key = "PROCEDURE_RESULT";  // Or TORQUE_ESTIMATE
            state_topic->values[5].key = "CONTROL_MODE";      // Or VEL_ESTIMATE
        } else {
            state_topic->values[1].key = "CONTROL_MODE";      // Or AXIS_ERROR
            state_topic->values[2].key = "INPUT_MODE";        // Or DISARM_REASON
            state_topic->values[3].key = "SETPOINT";          // Or ACTIVE_ERRORS
            state_topic->values[4].key = "TORQUE_ESTIMATE";   // Or PROCEDURE_RESULT
            state_topic->values[5].key = "VEL_ESTIMATE";      // Or CONTROL_MODE
        }
    }
    void update_diagnostics_label();

    void allocate_strings() {
        for (auto & string : strings) {
            string = new char[50];
            sprintf(string, "%s", "Unknown");
        }
    }
    /**
     * Updates the main diagnostics_topic string and the condition numbers
     */
    void update_diagnostics();


    void configure_diagnostics_topic(){
        this->state_topic->values_length = NUM_CONDITIONS;
        this->state_topic->values = new diagnostic_msgs::KeyValue[NUM_CONDITIONS];
        state_topic->values[0].key  = "AXIS_STATE";
        state_topic->values[1].key  = "AXIS_ERROR";        // Or CONTROL_MODE
        state_topic->values[2].key  = "ACTIVE_ERRORS";     // Or INPUT_MODE
        state_topic->values[3].key  = "DISARM_REASON";     // Or SETPOINT
        state_topic->values[4].key  = "PROCEDURE_RESULT";  // Or TORQUE_ESTIMATE
        state_topic->values[5].key  = "CONTROL_MODE";      // Or VEL_ESTIMATE
        state_topic->values[6].key  = "POS_ESTIMATE";
        state_topic->values[7].key  = "FET_TEMP";
        state_topic->values[8].key  = "MTR_TEMP";
        state_topic->values[9].key  = "BUS_VOLTAGE";
        state_topic->values[10].key = "BUS_CURRENT";
        state_topic->values[11].key = "MTR_CURRENT";       // AKA IQ_MEASURED
        state_topic->values[12].key = "ODOMETER";
        state_topic->name = "ODrive";
        state_topic->message = status_string;
        sprintf(status_string, "Initialising");
        state_topic->level = 0;
        state_topic->hardware_id = this->name.c_str();
        allocate_strings();
        for (int i = 0; i < NUM_CONDITIONS; i++) {
            this->state_topic->values[i].value = strings[i];
        }
    }

public:


    ODrive_ROS(ODrivePro* odrive,
               diagnostic_msgs::DiagnosticStatus* status,
               std_msgs::Int32MultiArray* encoder_topic,
               String disp_name) :
            setpoint_sub("template1", &ODrive_ROS::setpoint_callback, this) {
        this->odrive = odrive;
        this->output_topic = encoder_topic;
        this->output_topic->data_length = 6;
        this->output_topic->data = new int32_t[6];
        this->output_topic->data[0] = 0;  // POS_ESTIMATE
        this->output_topic->data[1] = 0;  // VEL_ESTIMATE
        this->output_topic->data[2] = 0;  // RAMP_RATE
        this->output_topic->data[3] = 0;  // CONTROL_MODE
        this->output_topic->data[4] = 0;  // AXIS_STATE
        this->output_topic->data[5] = 0;  // AXIS_ERROR
        this->state_topic = status;
        this->name = disp_name.c_str();
        this->configure_diagnostics_topic();
        this->setpoint_sub.topic_ = setpoint_topic_name;
        sprintf(setpoint_topic_name, "/mciu/%s/odrive/input", disp_name.c_str());
    }


    ODrivePro* get_odrive();

    void setpoint_callback(const std_msgs::Int32MultiArray &msg);

    void subscribe(ros::NodeHandle* node_handle) override;

    void update() override;

    static int32_t to_fixed_point(float value, float scale);

    static float from_fixed_point(int32_t value, float scale);
};


#endif //TEENSYCANTRANSCEIVER_ODRIVE_ROS_H
