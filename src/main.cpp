
#include <Arduino.h>
#include <Actuators/Actuators.h>
#include <FlexCAN_T4.h>
#include <ros.h>
#include "ODrive/ODrivePro.h"
#include "ODrive/ODrive_ROS.h"
#include "Actuators/ActuatorUnit.h"
#include "Actuators/ActuatorsROS.h"
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include "ROS_Publishers.h"
#include "ROS_Subscribers.h"
#include "ROS_Services.h"

#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticArray.h"
#include "Sensors/LoadCells.h"
#include "Sensors/BatteryMonitor.h"
#include "Odometers.h"
#include "Misc/HopperDoor.h"
#include "Misc/AccessoryPower.h"

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

#define SYSTEM_DIAGNOSTICS_COUNT 14
diagnostic_msgs::DiagnosticArray system_diagnostics;
diagnostic_msgs::DiagnosticStatus* system_info;
ros::Publisher sys_diag_pub("/diagnostics", &system_diagnostics);

//IntervalTimer load_cell_read_timer;

ODrivePro* odrives[6];
ODrive_ROS* odrive_ros[6];

ActuatorUnit* actuators[4];
ActuatorsROS* actuators_ros[4];
Actuators actuator_bus;

LoadCells* load_cells[2];

BatteryMonitor* battery_monitor;

ROSNode* ros_nodes[16];

Odometers odometers;

EStopController* e_stop_controller;
HopperDoor* hopper_door;
AccessoryPower* accessory_power;

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

void read_load_cells() {
    load_cells[0]->read();
    load_cells[1]->read();
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

uint32_t spin_time = 0;

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

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

    for (int i = 0; i < 6; i++) {
        odrives[i] = new ODrivePro(i, &can1, &node_handle);
//        odometers.reset_odometer(i);
        odrives[i]->pass_odometer_data(odometers.get_odometer(i));
    }

    odrives[5]->set_feedforward(&trencher_ff);

    for (ODrivePro* odrive : odrives) {
        if (odrive == nullptr) continue;
    }

    system_diagnostics.status_length = SYSTEM_DIAGNOSTICS_COUNT;
    system_diagnostics.status = new diagnostic_msgs::DiagnosticStatus[SYSTEM_DIAGNOSTICS_COUNT];

    odrive_ros[0] = new ODrive_ROS(odrives[0], &system_diagnostics.status[0],
                                   odrive_encoder_topics[0]->message, "Front_Left");
    odrive_ros[1] = new ODrive_ROS(odrives[1], &system_diagnostics.status[1],
                                   odrive_encoder_topics[1]->message, "Front_Right");
    odrive_ros[2] = new ODrive_ROS(odrives[2], &system_diagnostics.status[2],
                                   odrive_encoder_topics[2]->message, "Rear_Left");
    odrive_ros[3] = new ODrive_ROS(odrives[3], &system_diagnostics.status[3],
                                   odrive_encoder_topics[3]->message, "Rear_Right");
    odrive_ros[4] = new ODrive_ROS(odrives[5], &system_diagnostics.status[5],
                                   odrive_encoder_topics[4]->message, "Trencher");
    odrive_ros[5] = new ODrive_ROS(odrives[4], &system_diagnostics.status[4],
                                   odrive_encoder_topics[5]->message, "Conveyor");

    actuators[0] = new ActuatorUnit(&actuator_bus, 128); // Slot 3L
    actuators[1] = new ActuatorUnit(&actuator_bus, 129); // Slot 1L
    actuators[2] = new ActuatorUnit(&actuator_bus, 130); // Slot 2L
    actuators[3] = new ActuatorUnit(&actuator_bus, 131); // Slot 1R

    actuators_ros[0] = new ActuatorsROS(actuators[0], actuator_encoder_topics[0]->message,
                                        &system_diagnostics.status[6], "Front_Left");
    actuators_ros[1] = new ActuatorsROS(actuators[1], actuator_encoder_topics[1]->message,
                                        &system_diagnostics.status[7], "Front_Right");
    actuators_ros[2] = new ActuatorsROS(actuators[2], actuator_encoder_topics[2]->message,
                                        &system_diagnostics.status[8], "Rear_Left");
    actuators_ros[3] = new ActuatorsROS(actuators[3], actuator_encoder_topics[3]->message,
                                        &system_diagnostics.status[9], "Rear_Right");

    auto* load_cell_clk_pins =     new int[4] {25, 26, 27, 28};
    auto* load_cell_data_pins =    new int[4] {29, 30, 31, 32};
    auto* load_cell_calibrations = new float[4] {1.0, 1.0, 1.0, 1.0};
    load_cells[0] = new LoadCells(4, load_cell_clk_pins, load_cell_data_pins,
                                  load_cell_calibrations,
                                  &system_diagnostics.status[10], load_cell_topics[0]->message,
                                  "Hopper", 2048);
    auto* load_cell_clk_pins_2 =     new int[4] {34, 35, 36, 37};
    auto* load_cell_data_pins_2 =    new int[4] {38, 39, 40, 41};
    auto* load_cell_calibrations_2 = new float[4] {1.0, 1.0, 1.0, 1.0};
    load_cells[1] = new LoadCells(4, load_cell_clk_pins_2, load_cell_data_pins_2,
                                  load_cell_calibrations_2,
                                  &system_diagnostics.status[11], load_cell_topics[1]->message,
                                  "Suspension", load_cells[0]->get_used_eeprom());

    e_stop_controller = new EStopController();
    hopper_door = new HopperDoor();
    battery_monitor = new BatteryMonitor(&system_diagnostics.status[12], e_stop_controller);
    accessory_power = new AccessoryPower();

    for (auto & odrive : odrives) e_stop_controller->add_estop_device(odrive);
    for (auto & actuator : actuators) e_stop_controller->add_estop_device(actuator);

    // Add all ros nodes to the ros node array
    int ros_node_count = 0;
    for (auto & odrive_ro : odrive_ros) ros_nodes[ros_node_count++] = odrive_ro;
    for (auto & actuator_ro : actuators_ros) ros_nodes[ros_node_count++] = actuator_ro;
    for (auto & load_cell : load_cells) ros_nodes[ros_node_count++] = load_cell;
    ros_nodes[ros_node_count++] = e_stop_controller;
    ros_nodes[ros_node_count++] = hopper_door;
    ros_nodes[ros_node_count++] = battery_monitor;
    ros_nodes[ros_node_count++] = accessory_power;

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

    for(ros_topic* topic: all_topics) {
        if (topic == nullptr) continue;
        // Check if the topic's publisher is already in the node handle's PUBLISHER list
        node_handle.advertise(*topic->publisher);
    }

    for (ROSNode* node: ros_nodes) {
        if (node == nullptr) continue;
        node->subscribe(&node_handle);
    }

}

void loop() {

    uint32_t loop_start = micros(); // Get the time at the start of the loop
    digitalWriteFast(LED_BUILTIN, LOW); // Turn on the LED

    system_message_count = 0;
    for (char *string: system_status_messages) {
        string[0] = '\0';  // Clear the system status messages
    }
    system_info->level = diagnostic_msgs::DiagnosticStatus::OK;
    actuator_bus.sent_last_cycle = 0;

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
//            node->publish();
        }
    }

    // Calculate the bus voltage by averaging the voltages of all the ODrives
//    float_t bus_voltage = 0;
//    uint32_t odrive_count = 0;
//    for (ODrivePro *odrive: odrives) {
//        if (odrive == nullptr) continue;
//        float_t voltage = odrive->get_vbus_voltage();
//        if (isnanf(voltage)) continue;
//        bus_voltage += voltage;
//        odrive_count++;
//    }
//    if (odrive_count == 0) {
//        bus_voltage = NAN;  // If there are no ODrives connected set the bus voltage to NAN
//    } else bus_voltage /= odrive_count;
//    battery_monitor->update_bus_voltage(bus_voltage);

    e_stop_controller->update();

    odometers.refresh();  // Save the updated odometer data to the EEPROM if necessary

    system_diagnostics.header.stamp = node_handle.now(); // Update the timestamp of the diagnostics message
    system_diagnostics.header.seq++;  // Increment the sequence number for the diagnostics message

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
            actuator_bus.get_queue_size(),
            actuator_bus.total_messages_sent - actuator_bus.total_messages_received,
            actuator_bus.total_messages_received - actuator_bus.total_messages_processed);

    while (actuator_bus.spin(micros() - loop_start > 45000)) {
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
            actuator_bus.round_trip_time(),
            actuator_bus.sent_last_cycle);
    // Update the uptime
    sprintf(system_info_strings[8], "%.2luh %.2lum %.2lus",
            millis() / 3600000, (millis() / 60000) % 60, (millis() / 1000) % 60);

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

    sys_diag_pub.publish(&system_diagnostics);
}