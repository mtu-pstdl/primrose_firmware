//
// Created by Jay on 12/18/2023.
//

#ifndef PRIMROSE_MCIU_HARDWARE_OBJECTS_H
#define PRIMROSE_MCIU_HARDWARE_OBJECTS_H

#include "ODrive/ODrive_ROS.h"
#include "ROS_Publishers.h"

extern ODrive_ROS* odrive_ros[7];
extern ODrivePro*  odrives[7];

extern ActuatorsROS* actuators_ros[4];
extern ActuatorUnit* actuators[4];

extern Odometers odometers;
extern ros::NodeHandle node_handle;

extern FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> can1;

void allocate_odrives(){

    for (int i = 0; i < 7; i++) {
        odrives[i] = new ODrivePro(i, &can1, &node_handle);
        odrives[i]->pass_odometer_data(odometers.get_odometer(i));
    }

//    odrives[5]->set_feedforward(&trencher_ff);


    odrive_ros[0] = new ODrive_ROS(odrives[0],
                                   static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[0]->message),
                                   "Front_Left");
    odrive_ros[1] = new ODrive_ROS(odrives[1],
                                   static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[1]->message),
                                   "Front_Right");
    odrive_ros[2] = new ODrive_ROS(odrives[2],
                                   static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[2]->message),
                                   "Rear_Left");
    odrive_ros[3] = new ODrive_ROS(odrives[3],
                                   static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[3]->message),
                                   "Rear_Right");
    odrive_ros[4] = new ODrive_ROS(odrives[4],
                                   static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[4]->message),
                                   "Trencher");
    odrive_ros[5] = new ODrive_ROS(odrives[5],
                                   static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[5]->message),
                                   "Conveyor");
    odrive_ros[6] = new ODrive_ROS(odrives[6],
                                   static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[6]->message),
                                   "Hopper");
}

void allocate_actuators(){
    actuators[0] = new ActuatorUnit(128,
                                    new SteeringEncoders(0),
                                    new SuspensionEncoders(0x01)); // Slot 3L

    actuators[1] = new ActuatorUnit(129,
                                    new SteeringEncoders(4),
                                    new SuspensionEncoders(0x02)); // Slot 2R
    actuators[1]->set_inverted(true,0); // Set the motor to run in the opposite direction (for the conveyor
    actuators[1]->set_inverted(true,1);

    actuators[2] = new ActuatorUnit(130,
                                    new SteeringEncoders(10),
                                    new SuspensionEncoders(0x03)); // Slot 2L
    actuators[2]->set_inverted(true,0);
    actuators[2]->set_inverted(true,1);

    actuators[3] = new ActuatorUnit(131,
                                    new SteeringEncoders(24),
                                    new SuspensionEncoders(0x04)); // Slot 3R
    actuators[3]->set_inverted(true,0);

    actuators_ros[0] = new ActuatorsROS(actuators[0],
                                        static_cast<std_msgs::Int32MultiArray*>(actuator_encoder_topics[0]->message),
                                        "Front_Left");
    actuators_ros[1] = new ActuatorsROS(actuators[1],
                                        static_cast<std_msgs::Int32MultiArray*>(actuator_encoder_topics[1]->message),
                                        "Front_Right");
    actuators_ros[2] = new ActuatorsROS(actuators[2],
                                        static_cast<std_msgs::Int32MultiArray*>(actuator_encoder_topics[2]->message),
                                        "Rear_Left");
    actuators_ros[3] = new ActuatorsROS(actuators[3],
                                        static_cast<std_msgs::Int32MultiArray*>(actuator_encoder_topics[3]->message),
                                        "Rear_Right");
}

void allocate_hardware_objects(){
    allocate_odrives();
    allocate_actuators();
}

#endif //PRIMROSE_MCIU_HARDWARE_OBJECTS_H
