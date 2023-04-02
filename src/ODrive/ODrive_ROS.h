//
// Created by Jay on 12/20/2022.
//

#ifndef TEENSYCANTRANSCEIVER_ODRIVE_ROS_H
#define TEENSYCANTRANSCEIVER_ODRIVE_ROS_H


#include "odrive_constants.h"
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

class ODrive_ROS {

    ODrivePro *odrive = nullptr;
    String name = ""; // The name of the ODrive

private:

    ros::Subscriber<std_msgs::Float32MultiArray, ODrive_ROS> setpoint_sub;

    // Publishes the values of POS_ESTIMATE, VEL_ESTIMATE, IQ_SETPOINT, IQ_MEASURED
    std_msgs::Float32MultiArray* encoder_topic;

    // Publishes the values of AXIS_STATE, AXIS_ERROR, ACTIVE_ERRORS, DISARM_REASON
    diagnostic_msgs::DiagnosticStatus* state_topic;

#define NUM_CONDITIONS 13
    char* strings[NUM_CONDITIONS];
    char* status_string = new char[25];
    char* setpoint_topic_name = new char[25];

    void update_diagnostics_keys(bool error_mode){
        if (error_mode){
            state_topic->values[1].key = "AXIS_ERROR";        // Or CONTROL_MODE
            state_topic->values[2].key = "ACTIVE_ERRORS";     // Or SETPOINT
            state_topic->values[3].key = "DISARM_REASON";     // Or LAST_UPDATE
            state_topic->values[4].key = "PROCEDURE_RESULT";  // Or POS_ESTIMATE
            state_topic->values[5].key = "CONTROL_MODE";      // Or VEL_ESTIMATE
        } else {
            state_topic->values[1].key = "CONTROL_MODE";      // Or AXIS_ERROR
            state_topic->values[2].key = "LAST_UPDATE";       // Or DISARM_REASON
            state_topic->values[3].key = "SETPOINT";          // Or ACTIVE_ERRORS
            state_topic->values[4].key = "POS_ESTIMATE";      // Or PROCEDURE_RESULT
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
     * Updates the main status string and the condition numbers
     */
    void update_diagnostics();


    void configure_diagnostics_topic(){
        this->state_topic->values_length = NUM_CONDITIONS;
        this->state_topic->values = new diagnostic_msgs::KeyValue[NUM_CONDITIONS];
        state_topic->values[0].key = "AXIS_STATE";
        state_topic->values[1].key = "AXIS_ERROR";        // Or CONTROL_MODE
        state_topic->values[2].key = "ACTIVE_ERRORS";     // Or SETPOINT
        state_topic->values[3].key = "DISARM_REASON";     // Or LAST_UPDATE
        state_topic->values[4].key = "PROCEDURE_RESULT";  // Or POS_ESTIMATE
        state_topic->values[5].key = "CONTROL_MODE";      // Or VEL_ESTIMATE
        state_topic->values[6].key = "FET_TEMP";
        state_topic->values[7].key = "MOTOR_TEMP";
        state_topic->values[8].key = "VBUS_VOLTAGE";
        state_topic->values[9].key = "VBUS_CURRENT";
        state_topic->values[10].key = "IQ_SETPOINT";
        state_topic->values[11].key = "IQ_MEASURED";
        state_topic->values[12].key = "IN-FLIGHT";
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
               std_msgs::Float32MultiArray* encoder_topic,
               String disp_name) :
            setpoint_sub("template1", &ODrive_ROS::setpoint_callback, this) {
        this->odrive = odrive;
        this->encoder_topic = encoder_topic;
        this->encoder_topic->data_length = 5;
        this->encoder_topic->data = new float_t[5];
        this->encoder_topic->data[0] = 0;  // POS_ESTIMATE
        this->encoder_topic->data[1] = 0;  // VEL_ESTIMATE
        this->encoder_topic->data[2] = 0;  // RAMP_RATE
        this->encoder_topic->data[3] = 0;  // CONTROL_MODE
        this->encoder_topic->data[4] = 0;
        this->state_topic = status;
        this->name = disp_name.c_str();
        this->configure_diagnostics_topic();
        this->setpoint_sub.topic_ = setpoint_topic_name;
        sprintf(setpoint_topic_name, "/mciu/%s/setpoint", disp_name.c_str());

    }


    ODrivePro* get_odrive();

    void setpoint_callback(const std_msgs::Float32MultiArray &msg);

    void control_mode_callback(const std_msgs::Int32MultiArray &msg);

    void subscribe(ros::NodeHandle* node_handle);

    void update_all();
};


#endif //TEENSYCANTRANSCEIVER_ODRIVE_ROS_H
