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

#define TOPIC_BASE "/mciu/Actuators"

class ActuatorsROS : public ROSNode {

private:

    enum ros_commands {
        STOP = 0,
        SET_DUTY_CYCLE  = 1,
        SET_POSITION    = 2,
    };

    ros::Subscriber<std_msgs::Int32MultiArray, ActuatorsROS> command_sub;

    diagnostic_msgs::DiagnosticStatus* diagnostic_topic;
    std_msgs::Int32MultiArray* output_topic;

    String name;

    char* strings[9]{nullptr};
    char* status_string = new char[25];
    char* pub_name = new char[50];

    void allocate_strings() {
        for (auto & string : strings) {
            string = new char[25];
        }
    }

    ActuatorUnit* actuator;

    void update_status_message();

    void update_diagnostics_topic();


public:

    ActuatorsROS(ActuatorUnit* actuator, std_msgs::Int32MultiArray* output_topic,
                 diagnostic_msgs::DiagnosticStatus* status, String disp_name) :
            command_sub("template_for_later", &ActuatorsROS::control_callback, this){
        this->actuator = actuator;
        // Add key-value pairs to the condition topic
        sprintf(status_string, "Initializing");
        this->diagnostic_topic = status;
        this->name = disp_name;

        this->output_topic = output_topic;
        this->output_topic->data_length = 12;
        this->output_topic->data = new int32_t[12];

        this->diagnostic_topic->name = "ActuatorUnit";
        this->diagnostic_topic->hardware_id = this->name.c_str();
        this->diagnostic_topic->message = status_string;
        this->diagnostic_topic->level = 0;
        this->diagnostic_topic->values_length = 9;
        this->diagnostic_topic->values = new diagnostic_msgs::KeyValue[9];
        this->diagnostic_topic->values[0].key = "SUSP-Status";
        this->diagnostic_topic->values[1].key = "TURN-Status";
        this->diagnostic_topic->values[2].key = "SUSP-Position";
        this->diagnostic_topic->values[3].key = "TURN-Position";
        this->diagnostic_topic->values[4].key = "SUSP-Duty";
        this->diagnostic_topic->values[5].key = "TURN-Duty";
        this->diagnostic_topic->values[6].key = "Temperature";
        this->diagnostic_topic->values[7].key = "Currents";
        this->diagnostic_topic->values[8].key = "Voltages";
        allocate_strings();

        for (int i = 0; i < 9; i++) {
            this->diagnostic_topic->values[i].value = strings[i];
            sprintf(this->diagnostic_topic->values[i].value, "Initializing");
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

};


#endif //TEENSYCANTRANSCEIVER_ACTUATORSROS_H
