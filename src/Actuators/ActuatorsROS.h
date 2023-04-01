//
// Created by Jay on 3/20/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ACTUATORSROS_H
#define TEENSYCANTRANSCEIVER_ACTUATORSROS_H

#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Float32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/publisher.h"

#include "ActuatorUnit.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/KeyValue.h"

#define TOPIC_BASE "/mciu/Actuators"

class ActuatorsROS {

    const char* node_names[6][4] = {
            {TOPIC_BASE "/x00/condition", TOPIC_BASE "/x01/condition",
             TOPIC_BASE "/x02/condition",TOPIC_BASE "/x03/condition"},
            {TOPIC_BASE "/x00/encoder", TOPIC_BASE "/x01/encoder",
             TOPIC_BASE "/x02/encoder",TOPIC_BASE "/x03/encoder"},
            {TOPIC_BASE "/x00/state", TOPIC_BASE "/x01/state",
             TOPIC_BASE "/x02/state",TOPIC_BASE "/x03/state"},
            {TOPIC_BASE "/x00/setpoint", TOPIC_BASE "/x01/setpoint",
             TOPIC_BASE "/x02/setpoint",TOPIC_BASE "/x03/setpoint"},
            {TOPIC_BASE "/x00/control_mode", TOPIC_BASE "/x01/control_mode",
             TOPIC_BASE "/x02/control_mode",TOPIC_BASE "/x03/control_mode"}
    };

private:

    ros::NodeHandle* node_handle = nullptr; // The ROS node handle

    ros::Subscriber<std_msgs::Float32MultiArray, ActuatorsROS> setpoint_sub;

    ros::Publisher encoder_pub_;
//    ros::Publisher state_pub_;

    diagnostic_msgs::DiagnosticStatus* diagnostic_topic;
    std_msgs::Int32MultiArray encoder_topic;

    String name;

    char* strings[9];
    char* status_string = new char[25];

    void allocate_strings() {
        for (auto & string : strings) {
            string = new char[25];
        }
    }

    ActuatorUnit* actuator;

    void update_status_message();

    void update_diagnostics_topic();


public:

    ActuatorsROS(ActuatorUnit* actuator, uint8_t node_id, diagnostic_msgs::DiagnosticStatus* status,
                 String disp_name) :
            setpoint_sub(node_names[3][node_id], &ActuatorsROS::setpoint_callback, this),
            encoder_pub_(String(disp_name + "/encoders").c_str(), &encoder_topic){
        this->actuator = actuator;
        // Add key-value pairs to the condition topic
        sprintf(status_string, "Initializing");
        this->diagnostic_topic = status;
        this->name = disp_name;

        this->encoder_topic.data_length = 2;
        this->encoder_topic.data = new int32_t[2];

        this->diagnostic_topic->name = "ActuatorUnit";
        this->diagnostic_topic->message = status_string;
        this->diagnostic_topic->level = 0;
        this->diagnostic_topic->values_length = 9;
        this->diagnostic_topic->values = new diagnostic_msgs::KeyValue[9];
        this->diagnostic_topic->values[0].key = "M1-Status";
        this->diagnostic_topic->values[1].key = "M2-Status";
        this->diagnostic_topic->values[2].key = "M1-Position";
        this->diagnostic_topic->values[3].key = "M2-Position";
        this->diagnostic_topic->values[4].key = "M1-Velocity";
        this->diagnostic_topic->values[5].key = "M2-Velocity";
        this->diagnostic_topic->values[6].key = "Temperature";
        this->diagnostic_topic->values[7].key = "Currents";
        this->diagnostic_topic->values[8].key = "Voltages";
        allocate_strings();

        for (int i = 0; i < 9; i++) {
            this->diagnostic_topic->values[i].value = strings[i];
        }

        this->diagnostic_topic->hardware_id = this->name.c_str();
    }

    /**
     * This method sets up the ROS publishers and subscribers
     */
    void advertise_subscribe(ros::NodeHandle *nh);

    /**
     * This function is called when a message is received on the setpoint topic
     * @param msg The length of the message is 2
     */
    void setpoint_callback(const std_msgs::Float32MultiArray &msg);

    void control_mode_callback(const std_msgs::Int32MultiArray &msg);

    void update();

    void publish();

    void begin_homing();
};


#endif //TEENSYCANTRANSCEIVER_ACTUATORSROS_H
