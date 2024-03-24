//
// Created by Jay on 12/18/2023.
//

#ifndef PRIMROSE_MCIU_INITIALIZE_OBJECTS_H
#define PRIMROSE_MCIU_INITIALIZE_OBJECTS_H

#include "ODrive/ODriveROS.h"
#include "ROS_Publishers.h"
#include "Misc/HighSpeedLogger.h"

// Allocate space for the odrives using placement new
constexpr size_t num_odrives = 6;
uint8_t odrive_pro_memory[sizeof(ODrivePro) * num_odrives];  // NOLINT
uint8_t odrive_ros_memory[sizeof(ODriveROS) * num_odrives];  // NOLINT

extern ODriveROS* odrive_ros[num_odrives];  // NOLINT
extern ODrivePro* odrives[num_odrives];     // NOLINT

constexpr size_t num_actuators = 4;
uint8_t actuator_unit_memory[sizeof(ActuatorUnit) * num_actuators];  // NOLINT
uint8_t actuators_ros_memory[sizeof(ActuatorsROS) * num_actuators];  // NOLINT

extern ActuatorsROS* actuators_ros[num_actuators];  // NOLINT
extern ActuatorUnit* actuators[num_actuators];      // NOLINT

//constexpr size_t high_speed_logger_size = sizeof(HighSpeedLogger);
//uint8_t high_speed_logger_memory[high_speed_logger_size];  // NOLINT

// Misc objects

extern LoadCells* load_cells[2];            // NOLINT
extern BatteryMonitor* battery_monitor;     // NOLINT
extern IMU* imu_class;                      // NOLINT
extern EStopController* e_stop_controller;  // NOLINT
extern AccessoryPower* accessory_power;     // NOLINT
extern HopperDoor* hopper_door;             // NOLINT
extern SystemMonitor* system_monitor;       // NOLINT
//extern HighSpeedLogger* high_speed_logger;  // NOLINT

extern Odometers odometers;          // NOLINT
extern ros::NodeHandle node_handle;  // NOLINT

extern FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> can1;  // NOLINT

/**
 * Initializes the odrive classes and the odrive_ros classes in memory already allocated by the linker
 */
void allocate_odrives(){
    DROP_CRUMB_VALUE('ODRV', breadcrumb_type::CHAR4);

    for (int i = 0; i < num_odrives; i++) {
        DROP_CRUMB_VALUE(i, breadcrumb_type::INT);
        // Initialize the odrive's using placement new so that memory is allocated at compile time
        odrives[i] = new (&odrive_pro_memory[sizeof(ODrivePro) * i]) ODrivePro(i, &can1, &node_handle);
//        odrives[i]->pass_odometer_data(odometers.get_odometer(i));
    }

    DROP_CRUMB_VALUE('OROS', breadcrumb_type::CHAR4);

//    odrives[5]->set_feedforward(&trencher_ff);

    uint8_t odrive_num = 0;  // Keep track of position in the odrive_ros array
//    DROP_CRUMB_VALUE(&odrive_ros_memory, breadcrumb_type::ADDRESS);
    odrive_ros[0] = new
            ODriveROS(odrives[0],
                      static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[0]->message),
                      "Front_Left");
    odrive_ros[1] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[1],
                      static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[1]->message),
                      "Front_Right");
    odrive_ros[2] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[2],
                      static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[2]->message),
                      "Rear_Left");
    odrive_ros[3] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[3],
                      static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[3]->message),
                      "Rear_Right");
    odrive_ros[4] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[4],
                      static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[4]->message),
                      "Trencher");
    odrive_ros[5] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
            ODriveROS(odrives[5],
                      static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[5]->message),
                      "Conveyor");
//    odrive_ros[6] = new (&odrive_ros_memory[sizeof(ODriveROS) * odrive_num++])
//            ODriveROS(odrives[6],
//                      static_cast<std_msgs::Int32MultiArray*>(odrive_encoder_topics[6]->message),
//                      "Hopper");
    DROP_CRUMB_VALUE('DONE', breadcrumb_type::CHAR4);
}

/**
 * Initializes the actuator classes and the actuator_ros classes in memory already allocated by the linker
 * @note Uses placement new, but some sensor objects are not allocated at until runtime
 */
void allocate_actuators(){
    DROP_CRUMB_VALUE('ACTU', breadcrumb_type::CHAR4);
    uint8_t actuator_num = 0;  // Keep track of position in the actuators array
    actuators[0] = new (&actuator_unit_memory[sizeof(ActuatorUnit) * actuator_num++])
            ActuatorUnit(128,new SteeringEncoders(0),
                         new SuspensionEncoders(0x01));

    actuators[1] = new (&actuator_unit_memory[sizeof(ActuatorUnit) * actuator_num++])
            ActuatorUnit(129,new SteeringEncoders(4),
                         new SuspensionEncoders(0x02));

    actuators[1]->set_inverted(true,0); // Set the motor to run in the opposite direction
    actuators[1]->set_inverted(true,1);

    actuators[2] = new (&actuator_unit_memory[sizeof(ActuatorUnit) * actuator_num++])
            ActuatorUnit(130,new SteeringEncoders(10),
                         new SuspensionEncoders(0x03));
    actuators[2]->set_inverted(true,0);
    actuators[2]->set_inverted(true,1);

    actuators[3] = new (&actuator_unit_memory[sizeof(ActuatorUnit) * actuator_num++])
            ActuatorUnit(131,new SteeringEncoders(24),
                         new SuspensionEncoders(0x04));
    actuators[3]->set_inverted(true,0);

    actuator_num = 0;  // Reset the actuator_num variable
    actuators_ros[0] = new (&actuators_ros_memory[sizeof(ActuatorsROS) * actuator_num++])
            ActuatorsROS(actuators[0],
                         static_cast<std_msgs::Int32MultiArray*>(actuator_encoder_topics[0]->message),
                         "Front_Left");
    actuators_ros[1] = new (&actuators_ros_memory[sizeof(ActuatorsROS) * actuator_num++])
            ActuatorsROS(actuators[1],
                         static_cast<std_msgs::Int32MultiArray*>(actuator_encoder_topics[1]->message),
                         "Front_Right");
    actuators_ros[2] = new (&actuators_ros_memory[sizeof(ActuatorsROS) * actuator_num++])
            ActuatorsROS(actuators[2],
                         static_cast<std_msgs::Int32MultiArray*>(actuator_encoder_topics[2]->message),
                         "Rear_Left");
    actuators_ros[3] = new (&actuators_ros_memory[sizeof(ActuatorsROS) * actuator_num++])
            ActuatorsROS(actuators[3],
                         static_cast<std_msgs::Int32MultiArray*>(actuator_encoder_topics[3]->message),
                         "Rear_Right");
}

void allocate_misc_objects(){
    DROP_CRUMB_VALUE('MISC', breadcrumb_type::CHAR4);
    load_cells[0] = new LoadCells(0x05, "Suspen",
                                  static_cast<std_msgs::Int32MultiArray*>(load_cell_topics[0]->message));
    load_cells[1] = new LoadCells(0x06, "Hopper",
                                  static_cast<std_msgs::Int32MultiArray*>(load_cell_topics[1]->message));

    e_stop_controller = new EStopController(
            static_cast<std_msgs::String*>(estop_topic.message),
            static_cast<std_msgs::Int32MultiArray*>(all_topics[ESTOP_TOPIC_NUM]->message));
    battery_monitor = new BatteryMonitor(e_stop_controller,
                                         static_cast<sensor_msgs::BatteryState*>(all_topics[BATTERY_TOPIC_NUM]->message));
    imu_class = new IMU(static_cast<sensor_msgs::Imu*>(all_topics[IMU_TOPIC_NUM]->message));
    hopper_door = new HopperDoor();
    accessory_power = new AccessoryPower();
    system_monitor = new SystemMonitor(
            static_cast<std_msgs::Int32MultiArray*>(all_topics[SYSTEM_MONITOR_TOPIC_NUM]->message));
//    high_speed_logger = new (&high_speed_logger_memory)
//            HighSpeedLogger(static_cast<std_msgs::UInt32MultiArray*>(high_speed_logger_topic.message),
//                            high_speed_logger_topic.publisher);
}

void attach_estop_devices(){
    e_stop_controller->add_estop_device(&ADAU_BUS_INTERFACE);
    e_stop_controller->add_estop_device(system_monitor);
    for (auto & odrive : odrives) e_stop_controller->add_estop_device(odrive);
    for (auto & actuator : actuators) e_stop_controller->add_estop_device(actuator);
    for (auto & load_cell : load_cells) e_stop_controller->add_estop_device(load_cell);
    e_stop_controller->add_estop_device(battery_monitor);
    e_stop_controller->add_estop_device(imu_class);
}

/**
 * Initializes all of the objects that are allocated at compile time and allocates memory for the objects that are
 * allocated at runtime.
 * @note This function must be called after the ROS node has been initialized and before the main loop starts
 */
void setup_hardware_objects(){
    DROP_CRUMB();
    allocate_odrives();
    allocate_actuators();
    allocate_misc_objects();
}

#endif //PRIMROSE_MCIU_INITIALIZE_OBJECTS_H
