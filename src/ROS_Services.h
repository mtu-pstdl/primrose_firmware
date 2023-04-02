//
// Created by Jay on 4/2/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ROS_SERVICES_H
#define TEENSYCANTRANSCEIVER_ROS_SERVICES_H

#include <Arduino.h>
#include <ros.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_srvs/Empty.h"

extern ODrivePro* odrives[6];
extern ODrive_ROS* odrive_ros[6];
extern ActuatorUnit* actuators[4];
extern ActuatorsROS* actuators_ros[4];


void home_suspension_cb(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    for (ActuatorUnit* actuator : actuators) {
        actuator->set_control_mode(ActuatorUnit::control_modes::homing, 0);
    }
}
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> home_suspension("mciu/home_suspension",
                                                                                        &home_suspension_cb);

void home_steering_cb(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    for (ActuatorUnit* actuator : actuators) {
        actuator->set_control_mode(ActuatorUnit::control_modes::homing, 1);
    }
}
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> home_steering("mciu/home_steering",
                                                                                      &home_steering_cb);

void estop_cb(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    for (ActuatorUnit* actuator : actuators) {
        actuator->emergency_stop();
    }
    for (ODrivePro* odrive : odrives) {
        odrive->emergency_stop();
    }
    // Send the response
}
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> estop("mciu/emergency_stop",
                                                                             &estop_cb);

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response>* services[] = {
        &home_suspension,
        &home_steering,
        &estop
};

#endif //TEENSYCANTRANSCEIVER_ROS_SERVICES_H
