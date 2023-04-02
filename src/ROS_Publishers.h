//
// Created by Jay on 4/2/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ROS_PUBLISHERS_H
#define TEENSYCANTRANSCEIVER_ROS_PUBLISHERS_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

/**
 * All publishers need to be declared at compile time, so they can't be dynamically created within
 * the ODrive_ROS class nor the Actuator_ROS class. Instead, they are declared here and then
 * their pointers are passed to the ODrive_ROS and Actuator_ROS classes.
 */

std_msgs::Float32MultiArray odrive1_encoder_msg;
ros::Publisher odrive1_encoder_topic("mciu/front_left/encoder", &odrive1_encoder_msg);

std_msgs::Float32MultiArray odrive2_encoder_msg;
ros::Publisher odrive2_encoder_topic("mciu/front_right/encoder", &odrive2_encoder_msg);

std_msgs::Float32MultiArray odrive3_encoder_msg;
ros::Publisher odrive3_encoder_topic("mciu/rear_left/encoder", &odrive3_encoder_msg);

std_msgs::Float32MultiArray odrive4_encoder_msg;
ros::Publisher odrive4_encoder_topic("mciu/rear_right/encoder", &odrive4_encoder_msg);

std_msgs::Float32MultiArray odrive5_encoder_msg;
ros::Publisher odrive5_encoder_topic("mciu/trencher/encoder", &odrive5_encoder_msg);

std_msgs::Float32MultiArray odrive6_encoder_msg;
ros::Publisher odrive6_encoder_topic("mciu/conveyor/encoder", &odrive6_encoder_msg);

std_msgs::Float32MultiArray actuator1_encoder_msg;
ros::Publisher actuator1_encoder_topic("mciu/actuator1/encoder", &actuator1_encoder_msg);

std_msgs::Float32MultiArray actuator2_encoder_msg;
ros::Publisher actuator2_encoder_topic("mciu/actuator2/encoder", &actuator2_encoder_msg);

std_msgs::Float32MultiArray actuator3_encoder_msg;
ros::Publisher actuator3_encoder_topic("mciu/actuator3/encoder", &actuator3_encoder_msg);

std_msgs::Float32MultiArray actuator4_encoder_msg;
ros::Publisher actuator4_encoder_topic("mciu/actuator4/encoder", &actuator4_encoder_msg);

ros::Publisher* odrive_encoder_topics[6] = {
        &odrive1_encoder_topic,
        &odrive2_encoder_topic,
        &odrive3_encoder_topic,
        &odrive4_encoder_topic,
        &odrive5_encoder_topic,
        &odrive6_encoder_topic
};

std_msgs::Float32MultiArray* odrive_encoder_msgs[6] = {
        &odrive1_encoder_msg,
        &odrive2_encoder_msg,
        &odrive3_encoder_msg,
        &odrive4_encoder_msg,
        &odrive5_encoder_msg,
        &odrive6_encoder_msg
};

ros::Publisher* actuator_encoder_topics[4] = {
        &actuator1_encoder_topic,
        &actuator2_encoder_topic,
        &actuator3_encoder_topic,
        &actuator4_encoder_topic
};

std_msgs::Float32MultiArray* actuator_encoder_msgs[4] = {
        &actuator1_encoder_msg,
        &actuator2_encoder_msg,
        &actuator3_encoder_msg,
        &actuator4_encoder_msg
};

#endif //TEENSYCANTRANSCEIVER_ROS_PUBLISHERS_H
