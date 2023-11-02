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
#include "ROSNode.h"

#define TOPIC_BASE "/mciu/Actuator_Bus_Interface"

#define UNIT_SCALE 100

class ActuatorsROS : public ROSNode {

private:

    enum ROS_COMMANDS {
        STOP = 0,
        SET_POSITION = 1,
        SET_DUTY_CYCLE = 2,
    };

    ros::Subscriber<std_msgs::Int32MultiArray, ActuatorsROS> command_sub;

    std_msgs::Int32MultiArray* output_topic;

    String name;

    char* strings[9]{nullptr};
    char* pub_name = new char[50];

    ActuatorUnit* actuator;

public:

    ActuatorsROS(ActuatorUnit* actuator, std_msgs::Int32MultiArray* output_topic, String disp_name) :
            command_sub("template_for_later", &ActuatorsROS::control_callback, this){
        this->actuator = actuator;
        // Add key-value pairs to the condition topic
        this->name = disp_name;

        this->output_topic = output_topic;
        this->output_topic->data_length = 18;
        this->output_topic->data = new int32_t[18];
        if (disp_name == "Front_Left"){
            this->output_topic->data[0] = 0;
        } else if (disp_name == "Front_Right"){
            this->output_topic->data[0] = 1;
        } else if (disp_name == "Rear_Left"){
            this->output_topic->data[0] = 2;
        } else if (disp_name == "Rear_Right"){
            this->output_topic->data[0] = 3;
        }

        this->command_sub.topic_ = this->pub_name;
        sprintf(pub_name, "/mciu/%s/actuators/input", disp_name.c_str());
    }

    /**
     * This method sets up the ROS publishers and subscribers
     */
    void subscribe(ros::NodeHandle* node_handle) override;

    /**
     * This function is called when a serial_message is received on the setpoint topic
     * @param msg The length of the serial_message is 2
     */
    void control_callback(const std_msgs::Int32MultiArray &msg);

    void update() override;

    void publish() override;

    static int32_t to_fixed_point(float value, float scale);
};


#endif //TEENSYCANTRANSCEIVER_ACTUATORSROS_H
