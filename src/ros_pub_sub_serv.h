//
// Created by Jay on 4/1/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ROS_PUB_SUB_SERV_H
#define TEENSYCANTRANSCEIVER_ROS_PUB_SUB_SERV_H

#include "ros.h"

// Setup global publishers
diagnostic_msgs::DiagnosticArray system_diagnostics;

diagnostic_msgs::DiagnosticStatus* system_info;

ros::Publisher sys_diag_pub("/diagnostics", &system_diagnostics);

#endif //TEENSYCANTRANSCEIVER_ROS_PUB_SUB_SERV_H
