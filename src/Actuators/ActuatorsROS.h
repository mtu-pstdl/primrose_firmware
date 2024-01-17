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
        SET_VELOCITY = 3,
    };

    /**
     * ActuatorsROS::OutputArray is a named union for storing the output data into the Int32MultiArray message
     */
    union OutputArray {
        struct OutputData {
            int32_t actuator_id = 0;               // The serial ID of the actuator (static)
            int32_t m1_position = 0;               // Fixed point, x100 (unit: unknown)
            int32_t m1_velocity = 0;               // Fixed point, x100 (unit: unknown)
            int32_t m2_position = 0;               // Fixed point, x100 (unit: ticks)
            int32_t m2_velocity = 0;               // Fixed point, x100 (unit: ticks/s)
            int32_t m1_duty_cycle = 0;             // -100 - 100 (unit: %)
            int32_t m2_duty_cycle = 0;             // -100 - 100 (unit: %)
            int32_t m1_target_position = 0;        // Fixed point, x100 (unit: unknown)
            int32_t m2_target_position = 0;        // Fixed point, x100 (unit: ticks)
            int32_t m1_control_mode = 0;           // ActuatorUnit::control_modes
            int32_t m2_control_mode = 0;           // ActuatorUnit::control_modes
            int32_t m1_current = 0;                // Fixed point, x100 (unit: Amps)
            int32_t m2_current = 0;                // Fixed point, x100 (unit: Amps)
            int32_t m1_fault_flags = 0;            // ActuatorUnit::fault_flags bitmask
            int32_t m2_fault_flags = 0;            // ActuatorUnit::fault_flags bitmask
            int32_t main_battery_voltage = 0;      // Fixed point, x100 (unit: Volts)
            int32_t logic_battery_voltage = 0;     // Fixed point, x100 (unit: Volts)
            int32_t controller_temperature = 0;    // Fixed point, x100 (unit: Celsius)
            int32_t reserved[2] = {0, 0};  // Reserved for future use
        } data;
        int32_t raw_array[20];  // The raw array of data to be sent over the serial bus
    } output_data = {};


    union InputArray {
        struct InputData {
            ROS_COMMANDS command = STOP;
            int32_t target_actuator = 0;
            union Arguments {
                struct PositionArgs {
                    int32_t position = 0;
                } position_args;
                struct DutyCycleArgs {
                    int32_t duty_cycle = 0;
                } duty_cycle_args;
                struct VelocityArgs {
                    int32_t velocity = 0;
                } velocity_args;
            } arguments;
        } command_data;
        int32_t raw_array[sizeof (InputData) / sizeof (int32_t)] = {};
    };


    ros::Subscriber<std_msgs::Int32MultiArray, ActuatorsROS> command_sub;

    std_msgs::Int32MultiArray* output_topic;

    String name;

    char* strings[9]{nullptr};
    char* pub_name = new char[50];

    ActuatorUnit* actuator;

    uint32_t increment = 0;

public:

    ActuatorsROS(ActuatorUnit* actuator, std_msgs::Int32MultiArray* output_topic, String disp_name) :
            command_sub("template_for_later", &ActuatorsROS::control_callback, this){
        this->actuator = actuator;
        // Add key-value pairs to the condition topic
        this->name = disp_name;

        this->output_topic = output_topic;
        this->output_topic->data_length = sizeof (this->output_data.raw_array) / sizeof (int32_t);
        this->output_topic->data = this->output_data.raw_array;
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

    static int32_t to_fixed_point(float value, float scale);
};


#endif //TEENSYCANTRANSCEIVER_ACTUATORSROS_H
