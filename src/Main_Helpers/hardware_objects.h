//
// Created by Jay on 12/18/2023.
//

#ifndef PRIMROSE_MCIU_HARDWARE_OBJECTS_H
#define PRIMROSE_MCIU_HARDWARE_OBJECTS_H

#include "ODrive/ODriveROS.h"
#include "ROS_Publishers.h"

// Allocate space for the odrives using placement new
constexpr size_t num_odrives = 7;
uint8_t odrive_pro_memory[sizeof(ODrivePro) * num_odrives];  // NOLINT
uint8_t odrive_ros_memory[sizeof(ODriveROS) * num_odrives];  // NOLINT

extern ODriveROS* odrive_ros[num_odrives];  // NOLINT
extern ODrivePro* odrives[num_odrives];     // NOLINT

constexpr size_t num_actuators = 4;
uint8_t actuator_unit_memory[sizeof(ActuatorUnit) * num_actuators];  // NOLINT
uint8_t actuators_ros_memory[sizeof(ActuatorsROS) * num_actuators];  // NOLINT

extern ActuatorsROS* actuators_ros[num_actuators];  // NOLINT
extern ActuatorUnit* actuators[num_actuators];      // NOLINT

extern Odometers odometers;          // NOLINT
extern ros::NodeHandle node_handle;  // NOLINT

extern FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> can1;  // NOLINT

void allocate_odrives(){

    for (int i = 0; i < 7; i++) {
        // Initialize the odrive's using placement new so that memory is allocated at compile time
        odrives[i] = new (&odrive_pro_memory[sizeof(ODrivePro) * i]) ODrivePro(i, &can1, &node_handle);
        odrives[i]->pass_odometer_data(odometers.get_odometer(i));
    }

//    odrives[5]->set_feedforward(&trencher_ff);

    uint8_t odrive_num = 0;  // Keep track of position in the odrive_ros array
    odrive_ros[0] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[0], static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[0]->message),
                      "Front_Left");
    odrive_ros[1] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[1],static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[1]->message),
                      "Front_Right");
    odrive_ros[2] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[2],static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[2]->message),
                      "Rear_Left");
    odrive_ros[3] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[3],static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[3]->message),
                      "Rear_Right");
    odrive_ros[4] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[4],static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[4]->message),
                      "Trencher");
    odrive_ros[5] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[5],static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[5]->message),
                      "Conveyor");
    odrive_ros[6] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[6],static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[6]->message),
                      "Hopper");
}

void allocate_actuators(){

    uint8_t actuator_num = 0;  // Keep track of position in the actuators array
    actuators[0] = new (&actuator_unit_memory[sizeof(ActuatorUnit) * actuator_num++])
            ActuatorUnit(128,new SteeringEncoders(0),
                         new SuspensionEncoders(0x01)); // Slot 3L

    actuators[1] = new (&actuator_unit_memory[sizeof(ActuatorUnit) * actuator_num++])
            ActuatorUnit(129,new SteeringEncoders(4),
                         new SuspensionEncoders(0x02)); // Slot 2R

    actuators[1]->set_inverted(true,0); // Set the motor to run in the opposite direction
    actuators[1]->set_inverted(true,1);

    actuators[2] = new (&actuator_unit_memory[sizeof(ActuatorUnit) * actuator_num++])
            ActuatorUnit(130,new SteeringEncoders(10),
                         new SuspensionEncoders(0x03)); // Slot 2L
    actuators[2]->set_inverted(true,0);
    actuators[2]->set_inverted(true,1);

    actuators[3] = new (&actuator_unit_memory[sizeof(ActuatorUnit) * actuator_num++])
            ActuatorUnit(131,new SteeringEncoders(24),
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
