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

extern void home_suspension_cb(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> home_suspension("mciu/home_suspension",
                                                                                        &home_suspension_cb);

extern void home_steering_cb(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> home_steering("mciu/home_steering",
                                                                                      &home_steering_cb);

extern void estop_cb(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> estop("mciu/estop",
                                                                             &estop_cb);


ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response>* services[] = {
        &home_suspension,
        &home_steering,
        &estop
};

#endif //TEENSYCANTRANSCEIVER_ROS_SERVICES_H
