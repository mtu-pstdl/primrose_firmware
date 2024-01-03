//
// Created by Jay on 7/3/2023.
//

#ifndef PRIMROSE_MCIU_HOPPERDOOR_H
#define PRIMROSE_MCIU_HOPPERDOOR_H


#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32.h"
#include "EStopDevice.h"

//#define PWM_PIN
#define IN1_PIN 3
#define IN2_PIN 4

/**
 * The HopperDoor class takes data from the Hopper Door and publishes it to the ROS network.
 * @deprecated The hopper door is expected to be controlled by an actuator controller in the future.
 */
class HopperDoor : public ROSNode, public EStopDevice {

private:

    enum ros_commands {
        CLOSE = 0,
        STOP = 1,
        OPEN = 2,
    };

    ros::Subscriber<std_msgs::Int32, HopperDoor> hopper_door_sub;

    void hopper_door_callback(const std_msgs::Int32& msg);

public:

    HopperDoor() :
        hopper_door_sub("/mciu/Hopper/door", &HopperDoor::hopper_door_callback, this) {
//        pinMode(PWM_PIN, OUTPUT);
        pinMode(IN1_PIN, OUTPUT);
        pinMode(IN2_PIN, OUTPUT);
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
    }

    void subscribe(ros::NodeHandle* node_handle) override {
        node_handle->subscribe(hopper_door_sub);
    }

    void estop() override {
//        analogWrite(PWM_PIN, 0);
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
    }

};


#endif //PRIMROSE_MCIU_HOPPERDOOR_H
