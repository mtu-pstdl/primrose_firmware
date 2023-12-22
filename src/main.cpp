/**
 * @author Jay Sweeney, email: ajsweene@mtu.edu
 */

#include <Arduino.h>
#include <Actuators/Actuator_Bus_Interface.h>
#include <FlexCAN_T4.h>
#include <ros.h>
#include "ODrive/ODrivePro.h"
#include "ODrive/ODriveROS.h"
#include "Actuators/ActuatorUnit.h"
#include "Actuators/ActuatorsROS.h"
#include "ADAU_Interfaces/ADAU_Sensor.h"
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include "ROS_Publishers.h"

#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticArray.h"
#include "Sensor_nodes/LoadCells.h"
#include "Sensor_nodes/BatteryMonitor.h"
#include "Odometers.h"
#include "Misc/HopperDoor.h"
#include "Misc/AccessoryPower.h"
#include "Sensor_nodes/IMU.h"
#include "Sensors_internal/SteeringEncoders.h"
#include "Sensors_internal/SuspensionEncoders.h"
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/String.h"
#include "ADAU_Interfaces/ADAU_Tester.h"
#include "Watchdog_t4.h"
#include <EEPROM.h>
#include "Main_Helpers/build_info.h"
#include "Main_Helpers/initialize_objects.h"
#include "Main_Helpers/utility_functions.h"
#include "Main_Helpers/CrashParser.h"
#include "Misc/SystemMonitor.h"
#include "Main_Helpers/BreadCrumbs.h"

// If a loop takes longer than MAX_LOOP_TIME then the whole system will be reset so this is a hard limit
#define MAX_LOOP_TIME 1 // 1 second
WDT_T4<WDT1> wdt;

// Set to false if the system crashed during initialization during the last boot
uint32_t safe_mode_flag __attribute__((section(".noinit")));
uint32_t first_boot __attribute__((section(".noinit")));

ros::NodeHandle node_handle;
FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> can1;

std_msgs::String test_output_msg;
ros::Publisher test_output_pub("/test_output", &test_output_msg);

ODrivePro* odrives[num_odrives];
ODriveROS* odrive_ros[num_odrives];

ActuatorUnit* actuators[4];
ActuatorsROS* actuators_ros[4];

LoadCells* load_cells[2];

BatteryMonitor* battery_monitor;
IMU* imu_class;

ROSNode* ros_nodes[24];

Odometers odometers;

EStopController* e_stop_controller;
HopperDoor* hopper_door;
AccessoryPower* accessory_power;

ADAU_Tester* adauTester;

CrashParser parser;

int starting_actuator = 0;


void can_recieve(const CAN_message_t &msg) {
    // Check node ID (Upper 6 bits of CAN ID)
    uint8_t node_id = msg.id >> 5;
    for (ODrivePro* odrive : odrives) {
        if (odrive == nullptr) continue;
        if (odrive->get_can_id() == node_id) {
            odrive->on_message(msg);
        }
    }
}


uint8_t odometer_reset_sequence = 0;


/**
 * @brief Callback for the odometer reset subscriber
 * In order to prevent accidental resets of the odometer the user will need to send the values
 * 0x0FF1CE and 0x0DD1CE to the topic /odometer_reset in succession to reset the odometer
 * @param msg
 */
void odometer_reset_callback(const std_msgs::Int32& msg) {
    switch (odometer_reset_sequence) {
        case 0:
            if (msg.data == 0x0FF1CE) odometer_reset_sequence++;
            break;
        case 1:
            if (msg.data == 0x0DD1CE) {
                odometer_reset_sequence = 0;
                for (int i = 0; i < 14; i++){
                    odometers.reset_odometer(i);
                }
//                battery_monitor->reset_data();
            }
            break;
        default:
            odometer_reset_sequence = 0;
            break;
    }
}

// Setup ros subscriber for odometer reset it is an int32 message
ros::Subscriber<std_msgs::Int32> odometer_reset_sub("/odometer_reset", odometer_reset_callback);

enum start_flags {
    COLD_START,             // The system was started from a cold boot (power on)
    WARM_START,             // The system was started from a warm boot (reset)
    WATCHDOG_VIOLATION,     // The watchdog timer expired and the system was restarted automatically
    MEMORY_EXHAUSTION,      // The system ran out of memory in the heap and was restarted automatically
    SYSTEM_PANIC,           // The CPU encountered an unrecoverable error and restarted the system (hard fault)
    SETUP_FAILURE,          // The CPU encountered an error during setup and entered safe mode to prevent a boot loop
};
uint8_t startup_type = WARM_START;

enum safe_mode_flags : uint32_t {
    SAFE_MODE_ARM,          // The system is in safe mode and will not execute any hardware commands
    NORMAL_BOOT,            // The system is not in safe mode and will run as normal
    SAFE_MODE_EXIT,         // The system is in safe mode and will exit safe mode on the next boot
};

void setup() {

    // Check if this is the first boot
    if (first_boot != 0xDEADBEEF) {
        // If this is the first boot then set the execution allowed flag to true no matter what
        safe_mode_flag = NORMAL_BOOT;
        first_boot = 0xDEADBEEF;
        startup_type = COLD_START;
    }
    if (safe_mode_flag == SAFE_MODE_EXIT) safe_mode_flag = NORMAL_BOOT; // This is the exit condition for safe mode

    // Read the watchdog flag from eeprom
    uint8_t restart_flag = EEPROM.read(WATCHDOG_FLAG_ADDR);
    if (restart_flag == 0x5A) {
        // If the watchdog flag is set then the system was restarted by the watchdog
        // Clear the watchdog flag
        EEPROM.write(WATCHDOG_FLAG_ADDR, 0x00);
        startup_type = WATCHDOG_VIOLATION;
    } else if (restart_flag == 0x5B) {
        // If the memory exhaustion flag is set then the system ran out of memory
        // Clear the memory exhaustion flag
        EEPROM.write(WATCHDOG_FLAG_ADDR, 0x00);
        startup_type = MEMORY_EXHAUSTION;
    } else if (!safe_mode_flag) {
        startup_type = SETUP_FAILURE;
    } else if (parser) {
        startup_type = SYSTEM_PANIC;
    }

    node_handle.getHardware()->setBaud(4000000); // ~4Mbps
    node_handle.setSpinTimeout(100); // 50ms
    node_handle.initNode();
    node_handle.requestSyncTime();  // Sync time with ROS master
//    system_monitor = new SystemMonitor(
//            static_cast<std_msgs::UInt32MultiArray*>(all_topics[SYSTEM_MONITOR_TOPIC_NUM]->message));


    // If we are starting in safe mode exit here to prevent us from encountering the same error again and again
    if (!safe_mode_flag) return;
    safe_mode_flag = SAFE_MODE_ARM;  // Arm the safe mode flag in case initialization fails

    // Set up the CAN bus
    can1.begin();
    can1.setBaudRate(500000); // 500kbps
    can1.onReceive(can_recieve);

    can1.enableFIFO();
    can1.enableFIFOInterrupt();

    SPI.begin();
//    SPI1.begin();

    // Setup the odometer reset subscriber
    node_handle.subscribe(odometer_reset_sub);

    // Setup the test output publisher
    node_handle.advertise(test_output_pub);

    allocate_hardware_objects();  // Allocate memory for all the hardware objects

    load_cells[0] = new LoadCells(0x05, "Suspen",
                                  static_cast<std_msgs::Int32MultiArray*>(load_cell_topics[0]->message));
    load_cells[1] = new LoadCells(0x06, "Hopper",
                                  static_cast<std_msgs::Int32MultiArray*>(load_cell_topics[1]->message));

    e_stop_controller = new EStopController(
            static_cast<std_msgs::String*>(estop_topic.message),
            static_cast<std_msgs::Int32MultiArray*>(all_topics[ESTOP_TOPIC_NUM]->message));
    hopper_door = new HopperDoor();

    battery_monitor = new BatteryMonitor(e_stop_controller,
                                         static_cast<sensor_msgs::BatteryState*>(all_topics[BATTERY_TOPIC_NUM]->message));
    imu_class = new IMU(static_cast<sensor_msgs::Imu*>(all_topics[IMU_TOPIC_NUM]->message));

    accessory_power = new AccessoryPower();

    for (auto & odrive : odrives) e_stop_controller->add_estop_device(odrive);
    for (auto & actuator : actuators) e_stop_controller->add_estop_device(actuator);
    for (auto & load_cell : load_cells) e_stop_controller->add_estop_device(load_cell);
    e_stop_controller->add_estop_device(battery_monitor);
    e_stop_controller->add_estop_device(imu_class);
//    e_stop_controller->add_estop_device(system_monitor);

    // Add all ros nodes to the ros node array
    int ros_node_count = 0;
    for (auto & odrive : odrive_ros) ros_nodes[ros_node_count++] = odrive;
    for (auto & actuator : actuators_ros) ros_nodes[ros_node_count++] = actuator;
    for (auto & load_cell : load_cells) ros_nodes[ros_node_count++] = load_cell;
    ros_nodes[ros_node_count++] = e_stop_controller;
    ros_nodes[ros_node_count++] = hopper_door;
    ros_nodes[ros_node_count++] = battery_monitor;
    ros_nodes[ros_node_count++] = accessory_power;
    ros_nodes[ros_node_count++] = imu_class;
//    ros_nodes[ros_node_count++] = system_monitor;

    node_handle.advertise(*estop_topic.publisher);
    estop_topic.publisher->publish(estop_topic.message);


    for(ros_topic* topic: all_topics) {
        if (topic == nullptr) continue;
        // Check if the topic's publisher is already in the node handle's PUBLISHER list
        node_handle.advertise(*topic->publisher);
    }

    for (ROSNode* node: ros_nodes) {
        if (node == nullptr) continue;
        node->subscribe(&node_handle);
    }

    adauTester = new ADAU_Tester(&test_output_msg);

    test_output_msg.data = battery_monitor->debug_string;

    WDT_timings_t config;
    config.trigger = MAX_LOOP_TIME;
    config.timeout = 5;
    config.callback = watchdog_violation;
    wdt.begin(config);

    safe_mode_flag = NORMAL_BOOT;
}

void loop() {
    uint32_t loop_start = micros(); // Get the time at the start of the loop
    freeram();  // Calculate the amount of space left in the heap
    DROP_CRUMB();
    if (safe_mode_flag == NORMAL_BOOT) {

        adauTester->run();

        ADAU_BUS_INTERFACE.parse_buffer(); // Update the ADAU bus

        ACTUATOR_BUS_INTERFACE.sent_last_cycle = 0;

        // Get the teensy temperature
        check_temp();

        for (ODrivePro *odrive: odrives) {
            if (odrive == nullptr) continue;
            odrive->refresh_data();  // Get the latest data from the ODrive
        }

        // Every cycle start with a different actuator to prevent the same actuator from always being the first to update
        for (int i = 0; i < 4; i++) {
            uint32_t actuator_index = (i + starting_actuator) % 4;
            if (actuators[actuator_index] == nullptr) continue;
            actuators[actuator_index]->update();
        }
        starting_actuator = (starting_actuator + 1) % 4;

        if (node_handle.connected()) {
            for (ROSNode *node: ros_nodes) {
                if (node == nullptr) continue;
                node->update();
                node->publish();
            }
        }

        e_stop_controller->update();

        odometers.refresh();  // Save the updated odometer data to the EEPROM if necessary

        if (e_stop_controller->estop_message_updated())
            estop_topic.publisher->publish(estop_topic.message);
        for (ros_topic *topic: all_topics) {
            if (topic == nullptr) continue;
            topic->publisher->publish(topic->message);
        }

        test_output_pub.publish(&test_output_msg);

        while (ACTUATOR_BUS_INTERFACE.spin() && micros() - loop_start < 45000) {
            yield();  // Yield to other tasks
        }
    }

    String log_msg = "";
    int8_t spin_result = 0;
    char *line;
    spin_result = node_handle.spinOnce(); // 50ms timeout

    switch (spin_result) {
        case ros::SPIN_OK:
            break;
        case ros::SPIN_ERR:  // Always happens once on startup
            node_handle.logwarn("BUILD #" BUILD_NUMBER_STR " @ " BUILD_DATE " " BUILD_TIME);
            node_handle.logwarn("BUILD TYPE: " BUILD_TYPE);
            node_handle.logwarn("BUILD GIT HASH: " BUILD_GIT_HASH);
            node_handle.logwarn("BUILD GIT BRANCH: " BUILD_GIT_BRANCH);
            node_handle.logwarn("BUILD MACHINE NAME: " BUILD_MACHINE_NAME);
            switch (startup_type) {
                case COLD_START:
                    node_handle.logwarn("MCIU EXITED NORMALLY, COLD_START");
                    break;
                case WARM_START:
                    node_handle.logwarn("MCIU EXITED NORMALLY, WARM_START");
                    break;
                case WATCHDOG_VIOLATION:
                    node_handle.logerror("MCIU EXITED ABNORMALLY - CAUSE: WATCHDOG_VIOLATION");
                    break;
                case MEMORY_EXHAUSTION:
                    node_handle.logerror("MCIU EXITED ABNORMALLY - CAUSE: MEMORY_EXHAUSTION");
                    break;
                case SYSTEM_PANIC:
                    node_handle.logerror("MCIU EXITED ABNORMALLY - CAUSE: SYSTEM_PANIC");
                    parser.generate_crash_dump();
                    while ((line = parser.crash_dump()) != nullptr) {
                        node_handle.logerror(line);
                    }
                    break;
                // Safe mode exists to ensure that we can send the crash dump instead of just crashing again and again
                case SETUP_FAILURE:
                    node_handle.logerror("MCIU EXITED ABNORMALLY - CAUSE: SETUP_FAILURE");
                    parser.generate_crash_dump();
                    while ((line = parser.crash_dump()) != nullptr) {
                        node_handle.logerror(line);
                    }
                    node_handle.logerror("MCIU ENTERED SAFE MODE, NO HARDWARE INITIALIZED");
                    safe_mode_flag = SAFE_MODE_EXIT;
                    node_handle.logerror("MCIU MUST BE RESTARTED TO EXIT SAFE MODE");
                    break;
                default:
                    node_handle.logerror("MCIU EXITED ABNORMALLY - CAUSE: UNKNOWN");
                    break;
            }
            break;
        case ros::SPIN_TIMEOUT:
            log_msg = "Spin timeout";
            node_handle.logwarn(log_msg.c_str());
            break;
        default:
            log_msg = "Unknown spin result: " + String(spin_result);
            node_handle.logerror(log_msg.c_str());
            break;
    }

    uint32_t execution_time = micros() - loop_start;

    uint32_t loop_time = micros() - loop_start;
    if (loop_time > 50000) {

    } else {
        // Wait for the remaining time in the loop to maintain a 20Hz loop
        delayMicroseconds(50000 - loop_time);
    }

    wdt.feed();  // Feed the watchdog timer to prevent a reset
}