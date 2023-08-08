//
// Created by Jay on 7/8/2023.
//

#ifndef PRIMROSE_MCIU_ACCESSORYPOWER_H
#define PRIMROSE_MCIU_ACCESSORYPOWER_H

#include "Arduino.h"
#include "ROSNode.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"

#define LASER_PIN 2
#define CAMERA_PIN 1

class AccessoryPower : public ROSNode {

    enum accessory_ids {
        LASERS = 0,
        CAMERAS = 1,
    };

private:

        void accessory_power_callback(const std_msgs::Int32MultiArray& msg) {
            switch (msg.data[0]) {
                case LASERS:
                    digitalWriteFast(LASER_PIN, msg.data[1] ? HIGH : LOW);
                    break;
                case CAMERAS:
                    digitalWriteFast(CAMERA_PIN, msg.data[1] ? HIGH : LOW);
                    break;
            }
        }

        ros::Subscriber<std_msgs::Int32MultiArray, AccessoryPower> accessory_power_sub;


public:

    AccessoryPower() :
        accessory_power_sub("/mciu/accessory_power",
                            &AccessoryPower::accessory_power_callback, this) {
        pinMode(LASER_PIN, OUTPUT);
        pinMode(CAMERA_PIN, OUTPUT);
    }

    void subscribe(ros::NodeHandle* node_handle) override {
        node_handle->subscribe(accessory_power_sub);
    }

};


#endif //PRIMROSE_MCIU_ACCESSORYPOWER_H
