/**
 * @author Jay Sweeney, email: ajsweene@mtu.edu
 * The MCIU (Motor Controller Interface Unit) is the main controller for the PSTDL's PRIMROSE vehicle. It is responsible
 * for interfacing with all the hardware on the vehicle and providing a ROS interface for the higher level controllers
 * to use. It is also tasked with keeping the vehicle and those around it by ensuring that all hardware is operating
 * within safe limits and that the vehicle is being commanded in a safe manner. In the event of an emergency the MCIU
 * is responsible for bringing the vehicle to a safe stop by commanding all controllers to stop and disengaging the
 * high voltage power supply.
 * Utilizes the following hardware interfaces:
 * - CAN 1
 * - SPI 1 and 2
 * - UART 1, 2, 4
 * - I2C 1
 * Connected hardware:
 * - 1x Analog Data Acquisition Unit (ADAU, Additional Teensy 4.1 running in tandem with the MCIU)
 * - 6x ODrivePro running firmware v0.6.6
 * - 4x RoboClaw 2x60A motor controllers
 * - 2x 4 channel ADCs used for reading load cells (connected to the ADAU)
 * - 1x 4 channel ADC used for reading the linear encoders on the suspension (connected to the ADAU)
 * - 4x AMT22 absolute encoders used for reading the steering angle of each wheel
 * - 1x VE.Direct smart shunt for monitoring the battery
 * - 1x BNO055 IMU for measuring the orientation of the vehicle
 * @note This code is designed to run on a Teensy 4.1 microcontroller using the Arduino framework
 * @date Comment last updated: 2024/01/04 by Jay Sweeney
 */

#include <Arduino.h>
//#include <Wire.h>
//#include "../.pio/libdeps/teensy40/Teensy4 I2C/src/i2c_driver_wire.h"
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
#include "Main_Helpers/utility_functions.h"
#include "Main_Helpers/BreadCrumbs.h"
#include "Main_Helpers/CrashParser.h"
#include "Misc/SystemMonitor.h"
#include "Main_Helpers/initialize_objects.h"



// If a loop takes longer than MAX_LOOP_TIME then the whole system will be reset so this is a hard limit
#define MAX_LOOP_TIME 1 // 1 second
WDT_T4<WDT1> main_execution_watchdog;
WDT_T4<WDT2> setup_watchdog; // This watchdog is used to auto restart the teensy if a serial connection is not made

// Set to false if the system crashed during initialization during the last boot
uint32_t safe_mode_flag __attribute__((section(".noinit")));
uint32_t first_boot __attribute__((section(".noinit")));
uint32_t loop_count __attribute__((section(".noinit")));

ros::NodeHandle node_handle;

// CAN settings: 500kbps, 64 byte RX and TX buffers
FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> can1;

// TODO: Remove this for release build
std_msgs::String test_output_msg;
ros::Publisher test_output_pub("/test_output", &test_output_msg);

ODrivePro* odrives[num_odrives];
ODriveROS* odrive_ros[num_odrives];

ActuatorUnit* actuators[num_actuators];
ActuatorsROS* actuators_ros[num_actuators];

LoadCells* load_cells[2];

BatteryMonitor* battery_monitor;
IMU* imu_class;

EStopController* e_stop_controller;
HopperDoor* hopper_door;
AccessoryPower* accessory_power;
SystemMonitor* system_monitor;

ROSNode* ros_nodes[24];

Odometers odometers;

// TODO: Remove this for release build
ADAU_Tester* adauTester;

CrashParser parser;

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

/**
 * @warning The setup function will wait for a ROS Serial connection before returning
 */
void setup() {
    save_breadcrumbs(); // Copy the breadcrumbs from the previous boot into a separate buffer
    pinMode(MAIN_CONTACTOR_PIN, OUTPUT);
    digitalWrite(MAIN_CONTACTOR_PIN, HIGH);  // Immediately open the main contactor to initiate an estop

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
    } else {
        loop_count = 0;
    }

    node_handle.getHardware()->setBaud(4000000); // ~4Mbps
    node_handle.setSpinTimeout(100);
    node_handle.initNode();          // Initialize the ROS node (this will block until a connection is made)
    node_handle.requestSyncTime();   // Sync time with ROS master

    // If we are starting in safe mode exit here to prevent us from encountering the same error again and again
    if (!safe_mode_flag) return;
    safe_mode_flag = SAFE_MODE_ARM;  // Arm the safe mode flag in case initialization fails

    // Set up the CAN bus
    can1.begin();
    can1.setBaudRate(500000);       // Set the baud rate to 500kbps
    can1.onReceive(can_recieve);  // Set the callback function for when a CAN message is received

    can1.enableFIFO();                   // Enable the FIFO (First In First Out) buffer for the CAN bus
    can1.enableFIFOInterrupt();          // Allow the CAN bus to trigger an interrupt when a message is received

    SPI.begin();
//    SPI1.begin();

//    Wire.begin();

    // Setup the test output publisher
    node_handle.advertise(test_output_pub);

    setup_hardware_objects();  // Initialize all the hardware objects and allocate memory for the runtime objects

    attach_estop_devices();    // Attach all the estop devices to the estop controller

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
    ros_nodes[ros_node_count++] = system_monitor;

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

//    adauTester = new ADAU_Tester(&test_output_msg);

//    test_output_msg.data = battery_monitor->debug_string;

    test_output_msg.data = ADAU_BUS_INTERFACE.output_string;

    // The watchdog timers primary job is to reset the system if it somehow gets itself stuck into an infinite loop
    // It does not enforce the 20Hz loop time
    WDT_timings_t config;
    config.trigger = MAX_LOOP_TIME; // 1 second
    config.timeout = 5;
    config.callback = watchdog_violation;
    main_execution_watchdog.begin(config);
}

/**
 * Main MCIU Control Loop - Runs at 20Hz - RTCS Critical Section
 * @warning This is a real-time execution loop and all code must be non-blocking (no Delay() or busy waiting)
 * @warning All memory must be allocated at compile time or in the setup function to prevent memory fragmentation
 * @note This function is called by the Arduino framework and should not be called manually
 */
void loop() {
    uint32_t loop_start = micros(); // Get the time at the start of the loop
    freeram();  // Calculate the amount of space left in the heap
    DROP_CRUMB_VALUE(loop_count, breadcrumb_type::INT);
    if (safe_mode_flag == NORMAL_BOOT) {

        // TODO: Remove this for release build
//        adauTester->run();

        // Update the ADAU (Analogue Data Acquisition Unit) interface
        ADAU_BUS_INTERFACE.parse_buffer();

        ACTUATOR_BUS_INTERFACE.sent_last_cycle = 0;

        for (ODrivePro *odrive: odrives) {
            if (odrive == nullptr) continue;
            odrive->refresh_data();  // Send queries to the ODrives if they have not sent data on their own
        }

        static size_t starting_actuator = 0;
        // Every cycle start with a different actuator to prevent the same actuator from always being the first to update
        for (int i = 0; i < num_actuators; i++) {
            uint32_t actuator_index = (i + starting_actuator) % num_actuators;
            if (actuators[actuator_index] == nullptr) continue;
            actuators[actuator_index]->update();
        }
        starting_actuator = (starting_actuator + 1) % num_actuators;

        if (node_handle.connected()) {
            for (ROSNode *node: ros_nodes) {
                if (node == nullptr) continue;     // Skip over nullptrs
                if (node->dropped_task) continue;  // This node has violated the real-time constraints
                node->run_update();
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

        // TODO: Remove this for release build
        test_output_pub.publish(&test_output_msg);

        while (ACTUATOR_BUS_INTERFACE.spin() && micros() - loop_start < 45000) {
            // Can put other background tasks here if necessary, but they must be non-blocking
        }

        DROP_CRUMB();
    }

    String log_msg = "";
    int8_t spin_result;
    char *line;
    spin_result = node_handle.spinOnce(); // 50ms timeout

    switch (spin_result) {
        case ros::SPIN_OK:
            break;
        case ros::SPIN_ERR:  // Always happens once on startup. Reason unknown, but it is quite useful
//            setup_watchdog.reset();
            node_handle.logwarn("BUILD #" BUILD_NUMBER_STR " @ " BUILD_DATE " " BUILD_TIME);
            node_handle.logwarn("BUILD TYPE: " BUILD_TYPE);
            node_handle.logwarn("BUILD GIT HASH: " BUILD_GIT_HASH);
            node_handle.logwarn("BUILD GIT BRANCH: " BUILD_GIT_BRANCH);
            node_handle.logwarn("BUILD MACHINE NAME: " BUILD_MACHINE_NAME);
            switch (startup_type) {
                case COLD_START:
                    node_handle.logwarn("MCIU EXITED NORMALLY, COLD_START");
                    loop_count = 0;
                    safe_mode_flag = NORMAL_BOOT;
                    break;
                case WARM_START:
                    node_handle.logwarn("MCIU EXITED NORMALLY, WARM_START");
                    loop_count = 0;
                    safe_mode_flag = NORMAL_BOOT;
                    break;
                case WATCHDOG_VIOLATION:
                    node_handle.logerror("MCIU EXITED ABNORMALLY - CAUSE: WATCHDOG_VIOLATION");
                    parser.generate_crumb_dump();
                    while ((line = parser.crash_dump()) != nullptr) {
                        node_handle.logerror(line);
                    }
                    loop_count = 0;
                    safe_mode_flag = NORMAL_BOOT;
                    break;
                case MEMORY_EXHAUSTION:
                    node_handle.logerror("MCIU EXITED ABNORMALLY - CAUSE: MEMORY_EXHAUSTION");
                    parser.generate_crumb_dump();
                    while ((line = parser.crash_dump()) != nullptr) {
                        node_handle.logerror(line);
                    }
                    loop_count = 0;
                    safe_mode_flag = NORMAL_BOOT;
                    break;
                case SYSTEM_PANIC:
                    node_handle.logerror("MCIU EXITED ABNORMALLY - CAUSE: SYSTEM_PANIC");
                    parser.generate_crash_dump();
                    while ((line = parser.crash_dump()) != nullptr) {
                        node_handle.logerror(line);
                    }
                    loop_count = 0;
                    safe_mode_flag = NORMAL_BOOT;
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
                    loop_count = 0;
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
    if (execution_time < 50000) {
        // Wait for the remaining time in the loop to maintain a 20Hz loop
        delayMicroseconds(50000 - execution_time);
    }
    uint32_t loop_time = micros() - loop_start;

    // If the system monitor determines that the timing constraints have been violated then it will command
    // an emergency stop but will not reboot the system
    system_monitor->update_loop_info(execution_time, loop_time);  // Update the system monitor with the loop time

    main_execution_watchdog.feed();  // Feed the watchdog timer to prevent a reset
    loop_count++;
    DROP_CRUMB_VALUE('END ', breadcrumb_type::CHAR4);
}