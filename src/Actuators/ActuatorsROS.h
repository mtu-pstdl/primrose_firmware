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
    ros::Subscriber<std_msgs::Int32MultiArray, ActuatorsROS> control_mode_sub;

    ros::Publisher condition_pub_;
    ros::Publisher encoder_pub_;
    ros::Publisher state_pub_;

    diagnostic_msgs::DiagnosticStatus condition_topic;
    std_msgs::Int32MultiArray encoder_topic;
    std_msgs::Int32MultiArray state_topic;

    ActuatorUnit* actuator;

public:

    ActuatorsROS(ActuatorUnit* actuator, uint8_t node_id) :
            setpoint_sub(node_names[3][node_id], &ActuatorsROS::setpoint_callback, this),
            control_mode_sub(node_names[4][node_id], &ActuatorsROS::control_mode_callback, this),
            condition_pub_(node_names[0][node_id], &condition_topic),
            encoder_pub_(node_names[1][node_id], &encoder_topic),
            state_pub_(node_names[2][node_id], &state_topic){
        this->actuator = actuator;
        // Add key-value pairs to the condition topic
        this->condition_topic.values_length = 6;
        this->condition_topic.values = new diagnostic_msgs::KeyValue[6];
        this->condition_topic.values[0].key = "Temperature";
        this->condition_topic.values[1].key = "Current-M1";
        this->condition_topic.values[2].key = "Current-M2";
        this->condition_topic.values[3].key = "Main Volts";
        this->condition_topic.values[4].key = "Logic Volts";
        this->condition_topic.values[5].key = "Status";
        this->condition_topic.hardware_id = String("act" + String(node_id)).c_str();
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

};


#endif //TEENSYCANTRANSCEIVER_ACTUATORSROS_H
