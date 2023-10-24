//
// Created by Jay on 4/2/2023.
//

#ifndef TEENSYCANTRANSCEIVER_ROS_PUBLISHERS_H
#define TEENSYCANTRANSCEIVER_ROS_PUBLISHERS_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/sensor_msgs/Imu.h"
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/sensor_msgs/BatteryState.h"
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/msg.h"

/**
 * All publishers need to be declared at compile time, so they can't be dynamically created within
 * the ODrive_ROS class nor the Actuator_ROS class. Instead, they are declared here and then
 * their pointers are passed to the ODrive_ROS and Actuator_ROS classes.
 */

struct ros_topic {
    ros::Publisher* publisher;
    void* message;
};

std_msgs::Int32MultiArray odrive1_encoder_msg;
ros::Publisher odrive1_encoder_pub("mciu/Front_Left/odrive/output", &odrive1_encoder_msg);

#define ODRIVE1_ENCODER_TOPIC_NUM 0
ros_topic odrive1_encoder_topic = {
        .publisher = &odrive1_encoder_pub,
        .message = &odrive1_encoder_msg
};

std_msgs::Int32MultiArray odrive2_encoder_msg;
ros::Publisher odrive2_encoder_pub("mciu/Front_Right/odrive/output", &odrive2_encoder_msg);

#define ODRIVE2_ENCODER_TOPIC_NUM 1
ros_topic odrive2_encoder_topic = {
        .publisher = &odrive2_encoder_pub,
        .message = &odrive2_encoder_msg
};

std_msgs::Int32MultiArray odrive3_encoder_msg;
ros::Publisher odrive3_encoder_pub("mciu/Rear_Left/odrive/output", &odrive3_encoder_msg);

#define ODRIVE3_ENCODER_TOPIC_NUM 2
ros_topic odrive3_encoder_topic = {
        .publisher = &odrive3_encoder_pub,
        .message = &odrive3_encoder_msg
};

std_msgs::Int32MultiArray odrive4_encoder_msg;
ros::Publisher odrive4_encoder_pub("mciu/Rear_Right/odrive/output", &odrive4_encoder_msg);

#define ODRIVE4_ENCODER_TOPIC_NUM 3
ros_topic odrive4_encoder_topic = {
        .publisher = &odrive4_encoder_pub,
        .message = &odrive4_encoder_msg
};

std_msgs::Int32MultiArray odrive5_encoder_msg;
ros::Publisher odrive5_encoder_pub("mciu/Trencher/odrive/output", &odrive5_encoder_msg);

#define ODRIVE5_ENCODER_TOPIC_NUM 4
ros_topic odrive5_encoder_topic = {
        .publisher = &odrive5_encoder_pub,
        .message = &odrive5_encoder_msg
};

std_msgs::Int32MultiArray odrive6_encoder_msg;
ros::Publisher odrive6_encoder_pub("mciu/Conveyor/odrive/output", &odrive6_encoder_msg);

#define ODRIVE6_ENCODER_TOPIC_NUM 5
ros_topic odrive6_encoder_topic = {
        .publisher = &odrive6_encoder_pub,
        .message = &odrive6_encoder_msg
};

std_msgs::Int32MultiArray actuator1_encoder_msg;
ros::Publisher actuator1_encoder_pub("mciu/Front_Left/actuators/output", &actuator1_encoder_msg);

#define ACTUATOR1_ENCODER_TOPIC_NUM 6
ros_topic actuator1_encoder_topic = {
        .publisher = &actuator1_encoder_pub,
        .message = &actuator1_encoder_msg
};

std_msgs::Int32MultiArray actuator2_encoder_msg;
ros::Publisher actuator2_encoder_pub("mciu/Front_Right/actuators/output", &actuator2_encoder_msg);

#define ACTUATOR2_ENCODER_TOPIC_NUM 7
ros_topic actuator2_encoder_topic = {
        .publisher = &actuator2_encoder_pub,
        .message = &actuator2_encoder_msg
};

std_msgs::Int32MultiArray actuator3_encoder_msg;
ros::Publisher actuator3_encoder_pub("mciu/Rear_Left/actuators/output", &actuator3_encoder_msg);

#define ACTUATOR3_ENCODER_TOPIC_NUM 8
ros_topic actuator3_encoder_topic = {
        .publisher = &actuator3_encoder_pub,
        .message = &actuator3_encoder_msg
};

std_msgs::Int32MultiArray actuator4_encoder_msg;
ros::Publisher actuator4_encoder_pub("mciu/Rear_Right/actuators/output", &actuator4_encoder_msg);

#define ACTUATOR4_ENCODER_TOPIC_NUM 9
ros_topic actuator4_encoder_topic = {
        .publisher = &actuator4_encoder_pub,
        .message = &actuator4_encoder_msg
};

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/mciu/Imu", &imu_msg);

#define IMU_TOPIC_NUM 10
ros_topic imu_topic = {
        .publisher = &imu_pub,
        .message = &imu_msg
};

sensor_msgs::BatteryState battery_msg;
ros::Publisher battery_pub("/mciu/Battery", &battery_msg);

#define BATTERY_TOPIC_NUM 11
ros_topic battery_topic = {
        .publisher = &battery_pub,
        .message = &battery_msg
};

ros_topic* odrive_encoder_topics[6] = {
        &odrive1_encoder_topic,
        &odrive2_encoder_topic,
        &odrive3_encoder_topic,
        &odrive4_encoder_topic,
        &odrive5_encoder_topic,
        &odrive6_encoder_topic
};

ros_topic* actuator_encoder_topics[4] = {
        &actuator1_encoder_topic,
        &actuator2_encoder_topic,
        &actuator3_encoder_topic,
        &actuator4_encoder_topic
};

#define ALL_TOPICS_LENGTH 14
ros_topic* all_topics[ALL_TOPICS_LENGTH] = {
        &odrive1_encoder_topic,
        &odrive2_encoder_topic,
        &odrive3_encoder_topic,
        &odrive4_encoder_topic,
        &odrive5_encoder_topic,
        &odrive6_encoder_topic,
        &actuator1_encoder_topic,
        &actuator2_encoder_topic,
        &actuator3_encoder_topic,
        &actuator4_encoder_topic,
        &imu_topic,
        &battery_topic
};

#endif //TEENSYCANTRANSCEIVER_ROS_PUBLISHERS_H
