//
// Created by Jay on 7/3/2023.
//

#ifndef PRIMROSE_MCIU_HOPPERDOOR_H
#define PRIMROSE_MCIU_HOPPERDOOR_H


#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32.h"
#include "EStopDevice.h"

#define PWM_PIN 4
#define DIRECTION_PIN 3

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
        pinMode(PWM_PIN, OUTPUT);
        pinMode(DIRECTION_PIN, OUTPUT);
    }

    void subscribe(ros::NodeHandle* node_handle) override {
        node_handle->subscribe(hopper_door_sub);
    }

    void estop() override {
        analogWrite(PWM_PIN, 0);
        digitalWrite(DIRECTION_PIN, LOW);
    }

};


#endif //PRIMROSE_MCIU_HOPPERDOOR_H
