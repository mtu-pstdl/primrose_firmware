
#include <Arduino.h>
#include <Actuators/Actuator_Bus_Interface.h>
#include <FlexCAN_T4.h>
#include <ros.h>
#include "ODrive/ODrivePro.h"
#include "ODrive/ODrive_ROS.h"
#include "Actuators/ActuatorUnit.h"
#include "Actuators/ActuatorsROS.h"
#include "ADAU_Interfaces/ADAU_Sensor.h"
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include "ROS_Publishers.h"
#include "ROS_Subscribers.h"
#include "ROS_Services.h"

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

// Motor configurations
feedforward_struct trencher_ff = {
        true,
        19,
        new float_t[19]{100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900},
        new float_t[19]{0.8924418571359903, 0.9336526961981934, 0.9564353191563113, 1.0545682968419106, 1.0956791896783447, 1.1655547321087025, 1.1496691199748679, 1.1904955891573539, 1.2093394546417329, 1.2397901319373068, 1.28819931056913, 1.2661215055746053, 1.2726709948459896, 1.2938049218498964, 1.4171302597609259, 1.4186923515245125, 1.4106400849419782, 1.4540763298221129, 1.4360584135375623}
};

#define CPU_FREQ_BOOST 816000000 //
//#define CPU_FREQ_BASE 600000000 // 300 MHz
#define CPU_FREQ_BASE 300000000
//#define CPU_FREQ_BASE 24000000 // 300 MHz
#define CPU_FREQ_MIN 24000000 // 24 MHz
#define WARN_TEMP 65.0 // Degrees C
#define THROTTLE_TEMP 75.0 // Degrees C
uint32_t cpu_freq = CPU_FREQ_BASE;
uint32_t cpu_boost_time = 0;

ros::NodeHandle node_handle;
FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> can1;

#define SYSTEM_DIAGNOSTICS_COUNT 15
diagnostic_msgs::DiagnosticArray system_diagnostics;
diagnostic_msgs::DiagnosticStatus* system_info;
ros::Publisher sys_diag_pub("/diagnostics", &system_diagnostics);

std_msgs::String test_output_msg;
ros::Publisher test_output_pub("/test_output", &test_output_msg);
char test_output_string[100];

//IntervalTimer load_cell_read_timer;

ODrivePro* odrives[7];
ODrive_ROS* odrive_ros[7];

ActuatorUnit* actuators[4];
ActuatorsROS* actuators_ros[4];

LoadCells* load_cells[2];

BatteryMonitor* battery_monitor;
IMU* imu;

ROSNode* ros_nodes[16];

Odometers odometers;

EStopController* e_stop_controller;
HopperDoor* hopper_door;
AccessoryPower* accessory_power;

ADAU_Tester* adauTester;

//SteeringEncoders* steering_encoder;

#define SYSTEM_INFO_COUNT 10
char* system_info_strings[SYSTEM_INFO_COUNT];

char* system_status_messages[10];
char  system_status_msg[100];
uint8_t system_message_count = 0;

#if defined(__IMXRT1062__)
extern "C" uint32_t set_arm_clock(uint32_t frequency);
#endif

extern unsigned long _heap_start;  // start of heap
extern unsigned long _heap_end;  // end of heap
extern char *__brkval;  // current top of heap

uint32_t last_ram_time = 0;
int last_ram = 0;
int last_ram_usage = 0;

int starting_actuator = 0;

int freeram() {
    return (char *)&_heap_end - __brkval;
}

int ram_usage_rate(){
    int free_ram = freeram();
    int ram_usage = last_ram - free_ram;
    last_ram = free_ram;
    if (millis() - last_ram_time > 1000 || ram_usage > 0) {
        last_ram_time = millis();
        last_ram_usage = ram_usage;
    }
    return last_ram_usage;
}

void set_mciu_level_max(int8_t level) {
    if (level > system_diagnostics.status[SYSTEM_DIAGNOSTICS_COUNT - 1].level) {
        system_diagnostics.status[SYSTEM_DIAGNOSTICS_COUNT - 1].level = level;
    }
}

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


void check_temp(){
    if (tempmonGetTemp() > WARN_TEMP) {
        set_mciu_level_max(diagnostic_msgs::DiagnosticStatus::WARN);
        sprintf(system_status_messages[system_message_count++], "High temperature");
        if (tempmonGetTemp() > THROTTLE_TEMP) {
            set_mciu_level_max(diagnostic_msgs::DiagnosticStatus::ERROR);
            sprintf(system_status_messages[system_message_count++], "System throttling");
            set_arm_clock(CPU_FREQ_MIN);
        }
    } else set_arm_clock(cpu_freq);  // 600 MHz (default)
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
                battery_monitor->reset_data();
            }
            break;
        default:
            odometer_reset_sequence = 0;
            break;
    }
}

// Setup ros subscriber for odometer reset it is an int32 message
ros::Subscriber<std_msgs::Int32> odometer_reset_sub("/odometer_reset", odometer_reset_callback);

uint32_t spin_time = 0;

void setup() {

//    pinMode(LED_BUILTIN, OUTPUT);
//    digitalWrite(LED_BUILTIN, HIGH);

    node_handle.getHardware()->setBaud(4000000); // ~4Mbps
    node_handle.setSpinTimeout(100); // 50ms
    node_handle.initNode();
    node_handle.requestSyncTime();  // Sync time with ROS master

    // Set up the CAN bus
    can1.begin();
    can1.setBaudRate(500000); // 500kbps
    can1.onReceive(can_recieve);

    can1.enableFIFO();
    can1.enableFIFOInterrupt();

    SPI.begin();
    SPI1.begin();

    // Setup the odometer reset subscriber
    node_handle.subscribe(odometer_reset_sub);

    // Setup the test output publisher
    node_handle.advertise(test_output_pub);

//    steering_encoder = new SteeringEncoders(10);

    for (int i = 0; i < 7; i++) {
        odrives[i] = new ODrivePro(i, &can1, &node_handle);
        odrives[i]->pass_odometer_data(odometers.get_odometer(i));
    }

    odrives[5]->set_feedforward(&trencher_ff);

    for (ODrivePro* odrive : odrives) {
        if (odrive == nullptr) continue;
    }

    system_diagnostics.status_length = SYSTEM_DIAGNOSTICS_COUNT;
    system_diagnostics.status = new diagnostic_msgs::DiagnosticStatus[SYSTEM_DIAGNOSTICS_COUNT];

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
    imu = new IMU(static_cast<sensor_msgs::Imu*>(all_topics[IMU_TOPIC_NUM]->message));

    accessory_power = new AccessoryPower();

    for (auto & odrive : odrives) e_stop_controller->add_estop_device(odrive);
    for (auto & actuator : actuators) e_stop_controller->add_estop_device(actuator);
    for (auto & load_cell : load_cells) e_stop_controller->add_estop_device(load_cell);
    e_stop_controller->add_estop_device(battery_monitor);
    e_stop_controller->add_estop_device(imu);

    // Add all ros nodes to the ros node array
    int ros_node_count = 0;
    for (auto & odrive : odrive_ros) ros_nodes[ros_node_count++] = odrive;
    for (auto & actuator : actuators_ros) ros_nodes[ros_node_count++] = actuator;
    for (auto & load_cell : load_cells) ros_nodes[ros_node_count++] = load_cell;
    ros_nodes[ros_node_count++] = e_stop_controller;
    ros_nodes[ros_node_count++] = hopper_door;
    ros_nodes[ros_node_count++] = battery_monitor;
    ros_nodes[ros_node_count++] = accessory_power;
    ros_nodes[ros_node_count++] = imu;

    // Allocate memory for the system diagnostics strings
    for (auto & system_info_string : system_info_strings) system_info_string = new char[55];
    for (auto & system_status_message : system_status_messages) system_status_message = new char[20];

    system_info = &system_diagnostics.status[SYSTEM_DIAGNOSTICS_COUNT - 1];
    system_info->values_length = SYSTEM_INFO_COUNT;
    system_info->values = new diagnostic_msgs::KeyValue[SYSTEM_INFO_COUNT];
    system_info->values[0].key = "Temperature";
    system_info->values[1].key = "System Freq";
    system_info->values[2].key = "System Load";
    system_info->values[3].key = "Loop Time";
    system_info->values[4].key = "Remaining Memory";
    system_info->values[5].key = "CAN TX Overflow";
    system_info->values[6].key = "Actuator Buffer Info";
    system_info->values[7].key = "Actuator Response Time";
    system_info->values[8].key = "MCIU Uptime";
    system_info->values[9].key = "Firmware Build Date";
    sprintf(system_info_strings[9], "%s, %s", __DATE__, __TIME__);
    system_info->level = diagnostic_msgs::DiagnosticStatus::OK;
    system_info->name = "System";
    system_info->message = system_status_msg;
    system_info->hardware_id = "MCIU";

    for (int i = 0; i < SYSTEM_INFO_COUNT; i++) system_info->values[i].value = system_info_strings[i];
    // For each ODrive add its diagnostic serial_message

    node_handle.advertise(sys_diag_pub);
    sys_diag_pub.publish(&system_diagnostics);

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

    node_handle.loginfo("Running Firmware Build: " __DATE__ " " __TIME__);
}

void loop() {

    uint32_t loop_start = micros(); // Get the time at the start of the loop
//    digitalWriteFast(LED_BUILTIN, LOW); // Turn on the LED

    adauTester->run();

    ADAU_BUS_INTERFACE.parse_buffer(); // Update the ADAU bus

    system_message_count = 0;
    for (char *string: system_status_messages) {
        string[0] = '\0';  // Clear the system status messages
    }
    system_info->level = diagnostic_msgs::DiagnosticStatus::OK;
    ACTUATOR_BUS_INTERFACE.sent_last_cycle = 0;

    // Get the teensy temperature

    sprintf(system_info_strings[0], "%.1fC", tempmonGetTemp());
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


    // Calculate the bus voltage by averaging the voltages of all the ODrives
    float_t bus_voltage = 0;
    uint32_t odrive_count = 0;
    for (ODrivePro *odrive: odrives) {
        if (odrive == nullptr) continue;
        float_t voltage = odrive->get_vbus_voltage();
        if (isnanf(voltage)) continue;
        bus_voltage += voltage;
        odrive_count++;
    }
    if (odrive_count == 0) {
        bus_voltage = NAN;  // If there are no ODrives connected set the bus voltage to NAN
    } else bus_voltage /= odrive_count;
    battery_monitor->update_bus_voltage(bus_voltage);

    e_stop_controller->update();

    odometers.refresh();  // Save the updated odometer data to the EEPROM if necessary

    system_diagnostics.header.stamp = node_handle.now(); // Update the timestamp of the diagnostics message
    system_diagnostics.header.seq++;  // Increment the sequence number for the diagnostics message

    if (e_stop_controller->estop_message_updated())
        estop_topic.publisher->publish(estop_topic.message);
    for (ros_topic *topic: all_topics) {
        if (topic == nullptr) continue;
        topic->publisher->publish(topic->message);
    }

    String log_msg = "";
    int8_t spin_result = 0;
    spin_result = node_handle.spinOnce(); // 50ms timeout

    switch (spin_result) {
        case ros::SPIN_OK:
            break;
        case ros::SPIN_ERR:
            log_msg = "Spin error";
            node_handle.logerror(log_msg.c_str());
            // Collect info about the error
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
    // Allow the actuator bus to preform serial communication for the remaining time in the loop
    sprintf(system_info_strings[6], "S:%02d, F:%05lu, E:%05lu",
            ACTUATOR_BUS_INTERFACE.get_queue_size(),
            ACTUATOR_BUS_INTERFACE.total_messages_sent - ACTUATOR_BUS_INTERFACE.total_messages_received,
            ACTUATOR_BUS_INTERFACE.total_messages_received - ACTUATOR_BUS_INTERFACE.total_messages_processed);

    while (ACTUATOR_BUS_INTERFACE.spin() && micros() - loop_start < 45000) {
        yield();  // Yield to other tasks
    }

    uint32_t remaining_memory = freeram();

    if (ram_usage_rate() > 100) {
        set_mciu_level_max(diagnostic_msgs::DiagnosticStatus::WARN);
        sprintf(system_status_messages[system_message_count++], "Memory Leak");
    }

    if (remaining_memory < 100000) {
        if (remaining_memory < 30000) {
            set_mciu_level_max(diagnostic_msgs::DiagnosticStatus::ERROR);
            sprintf(system_status_messages[system_message_count++], "Out of Memory");
        }
        set_mciu_level_max(diagnostic_msgs::DiagnosticStatus::WARN);
        sprintf(system_status_messages[system_message_count++], "Low Memory");
    }
    uint32_t execution_time = micros() - loop_start;
    sprintf(system_info_strings[1], "%.2f Mhz (%.2f%%)", F_CPU_ACTUAL / 1000000.0,
            100.0 * F_CPU_ACTUAL / F_CPU);
    sprintf(system_info_strings[2], "%.5luus (%05.2f%%)", execution_time,
            (execution_time / 50000.0) * 100.0);
    sprintf(system_info_strings[4], "%.2fKiB (%.2f%%), %dKib/s", remaining_memory / 1024.0,
            100.0 * remaining_memory / 512000.0, ram_usage_rate());
    sprintf(system_info_strings[5], "%lu", can1.getTXQueueCount());
    sprintf(system_info_strings[7], "%lums (Sent %lu)",
            ACTUATOR_BUS_INTERFACE.round_trip_time(),
            ACTUATOR_BUS_INTERFACE.sent_last_cycle);
    // Update the uptime
    sprintf(system_info_strings[8], "%.2luh %.2lum %.2lus",
            millis() / 3600000, (millis() / 60000) % 60, (millis() / 1000) % 60);

    if (can1.getTXQueueCount() > 5){
        set_mciu_level_max(diagnostic_msgs::DiagnosticStatus::WARN);
        sprintf(system_status_messages[system_message_count++], "CAN BUS Saturation");
    }

    uint32_t loop_time = micros() - loop_start;
    if (loop_time > 50000) {
        set_mciu_level_max(diagnostic_msgs::DiagnosticStatus::WARN);
        sprintf(system_status_messages[system_message_count++], "Slow Loop");
        // Set the CPU to the overclocked velocity
    } else {
        // Wait for the remaining time in the loop to maintain a 20Hz loop
        delayMicroseconds(50000 - loop_time);
    }

    sprintf(system_info_strings[3], "%0.5luus (%05.2fHz)", micros() - loop_start, 1000000.0 / (micros() - loop_start));

    if (system_message_count == 0) {
        sprintf(system_status_msg, "All Ok");
    } else {
        for (int i = 0; i < system_message_count; i++) {
            if (i == 0) {
                sprintf(system_status_msg, "%s", system_status_messages[i]);
            } else {
                sprintf(system_status_msg, "%s, %s", system_status_msg, system_status_messages[i]);
            }
        }
    }

//    sys_diag_pub.publish(&system_diagnostics);
}