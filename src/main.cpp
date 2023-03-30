
#include <Arduino.h>
#include <Actuators/Actuators.h>
#include <FlexCAN_T4.h>
#include <ros.h>
//#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include "ODrive/ODriveS1.h"
#include "ODrive/ODrive_ROS.h"
#include "Actuators/ActuatorUnit.h"
#include "Actuators/ActuatorsROS.h"
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>

//#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Float32.h"
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticArray.h"
//#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/publisher.h"
//#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Float32.h"

#define CPU_FREQ_BOOST 816000000 //
//#define CPU_FREQ_BASE 600000000 // 300 MHz
#define CPU_FREQ_BASE 24000000 // 300 MHz
#define CPU_FREQ_MIN 24000000 // 24 MHz
#define UN_BOOST_TIME 100000 // 10us
//#define ENABLE_BOOST
#define WARN_TEMP 65.0 // Degrees C
#define THROTTLE_TEMP 75.0 // Degrees C
uint32_t cpu_freq = CPU_FREQ_BASE;
uint32_t cpu_boost_time = 0;

ros::NodeHandle node_handle;
FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> can1;

//#define HIGH_SPEED_USB

ODriveS1* odrives[6];
ODrive_ROS* odrive_ros[6];

ActuatorUnit* actuators[4];
ActuatorsROS* actuators_ros[4];
Actuators actuator_bus;

// Setup global publishers
diagnostic_msgs::DiagnosticArray system_diagnostics;

diagnostic_msgs::DiagnosticStatus* system_info;

ros::Publisher sys_diag_pub("/diagnostics", &system_diagnostics);

#define SYSTEM_INFO_COUNT 8
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

int freeram() {
    return (char *)&_heap_end - __brkval;
}

void can_recieve(const CAN_message_t &msg) {
    // Check node ID (Upper 6 bits of CAN ID)
    uint8_t node_id = msg.id >> 5;
//    Serial.printf("CAN event %d\n", node_id);
    for (ODriveS1* odrive : odrives) {
        if (odrive == nullptr) continue;
        if (odrive->get_can_id() == node_id) {
            odrive->on_message(msg);
        }
    }
}



void unified_estop_callback(){
    for (ODriveS1* odrive : odrives) {
        if (odrive == nullptr) continue;
        odrive->estop();
    }
    for (ActuatorUnit* actuator : actuators) {
        if (actuator == nullptr) continue;
        actuator->estop();
    }
}

void check_temp(){
    if (tempmonGetTemp() > WARN_TEMP) {
        system_diagnostics.status[10].level = diagnostic_msgs::DiagnosticStatus::WARN;
        sprintf(system_status_messages[system_message_count++], "High temperature");
        if (tempmonGetTemp() > THROTTLE_TEMP) {
            system_diagnostics.status[10].level = diagnostic_msgs::DiagnosticStatus::ERROR;
            sprintf(system_status_messages[system_message_count++], "System throttling");
            set_arm_clock(CPU_FREQ_MIN);
        }
    } else set_arm_clock(cpu_freq);  // 600 MHz (default)
}

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(1000);

#ifdef HIGH_SPEED_USB
//    node_handle.getHardware()->setBaud(500000); // 500kbps
#else
    node_handle.getHardware()->setBaud(4000000); // 4Mbps
#endif
    node_handle.setSpinTimeout(100); // 50ms
    node_handle.initNode();
    node_handle.requestSyncTime();

    String log_msg = "Starting MCIU with build version: " + String(__DATE__) + " " + String(__TIME__);
    node_handle.loginfo(log_msg.c_str());

    log_msg = "Initialising CAN bus at 500kbps";
    node_handle.loginfo(log_msg.c_str());

    // Set up the CAN bus
    can1.begin();
    can1.setBaudRate(500000); // 500kbps
//    can1.setMaxMB(64);  // 64 message buffers
    can1.onReceive(can_recieve);

    can1.enableFIFO();
    can1.enableFIFOInterrupt();

    log_msg = "CAN bus initialised";
    node_handle.loginfo(log_msg.c_str());

    log_msg = "Initialising ODriveS1 objects";
    node_handle.loginfo(log_msg.c_str());

    for (int i = 0; i < 6; i++) {
        odrives[i] = new ODriveS1(i, &can1, &node_handle);
    }

    for (ODriveS1* odrive : odrives) {
        if (odrive == nullptr) continue;
        odrive->set_conversion(172892, 0.92);
    }

    system_diagnostics.status_length = 11;
    system_diagnostics.status = new diagnostic_msgs::DiagnosticStatus[11];

    log_msg = "Initialising ODriveS1 ROS objects";
    node_handle.loginfo(log_msg.c_str());

    odrive_ros[0] = new ODrive_ROS(odrives[0], &node_handle, &system_diagnostics.status[0], "Front Left");
    odrive_ros[1] = new ODrive_ROS(odrives[1], &node_handle, &system_diagnostics.status[1], "Front Right");
    odrive_ros[2] = new ODrive_ROS(odrives[2], &node_handle, &system_diagnostics.status[2], "Rear Left");
    odrive_ros[3] = new ODrive_ROS(odrives[3], &node_handle, &system_diagnostics.status[3], "Rear Right");
    odrive_ros[4] = new ODrive_ROS(odrives[4], &node_handle, &system_diagnostics.status[4], "Conveyor");
    odrive_ros[5] = new ODrive_ROS(odrives[5], &node_handle, &system_diagnostics.status[5], "Trencher");

    log_msg = "Initialising ActuatorUnit objects";
    node_handle.loginfo(log_msg.c_str());

    actuators[0] = new ActuatorUnit(&actuator_bus, 0x80);
    actuators[1] = new ActuatorUnit(&actuator_bus, 0x81);
    actuators[2] = new ActuatorUnit(&actuator_bus, 0x82);
    actuators[3] = new ActuatorUnit(&actuator_bus, 0x83);

    log_msg = "Initialising ActuatorUnit ROS objects";
    node_handle.loginfo(log_msg.c_str());

    actuators_ros[0] = new ActuatorsROS(actuators[0], &node_handle, &system_diagnostics.status[6], "Front Left");
    actuators_ros[1] = new ActuatorsROS(actuators[1], &node_handle, &system_diagnostics.status[7], "Front Right");
    actuators_ros[2] = new ActuatorsROS(actuators[2], &node_handle, &system_diagnostics.status[8], "Rear Left");
    actuators_ros[3] = new ActuatorsROS(actuators[3], &node_handle, &system_diagnostics.status[9], "Rear Right");

    for (ODrive_ROS* odrive : odrive_ros) {
        if (odrive == nullptr) continue;
        log_msg = "Advertising ODrive";
        node_handle.loginfo(log_msg.c_str());
        odrive->advertise_subscribe(&node_handle);
    }

    for (ActuatorsROS* actuator : actuators_ros) {
        if (actuator == nullptr) continue;
        log_msg = "Advertising Actuator";
        node_handle.loginfo(log_msg.c_str());
        actuator->advertise_subscribe(&node_handle);
    }

    log_msg = "Setting up system diagnostics";
    node_handle.loginfo(log_msg.c_str());

    // Allocate memory for the system diagnostics strings
    for (auto & system_info_string : system_info_strings) system_info_string = new char[20];
    for (auto & system_status_message : system_status_messages) system_status_message = new char[20];

    system_info = &system_diagnostics.status[10];
    system_info->values_length = SYSTEM_INFO_COUNT;
    system_info->values = new diagnostic_msgs::KeyValue[SYSTEM_INFO_COUNT];
    system_info->values[0].key = "Temperature";
    system_info->values[1].key = "System Freq";
    system_info->values[2].key = "Loop Time";
    system_info->values[3].key = "Remaining Memory";
    system_info->values[4].key = "CAN TX Overflow";
    system_info->values[5].key = "CAN RX Overflow";
    system_info->values[6].key = "Actuator response time";
    system_info->values[7].key = "Build Date";
    for (int i = 0; i < SYSTEM_INFO_COUNT; i++) system_info->values[i].value = system_info_strings[i];
    sprintf(system_info_strings[7], "%s, %s", __DATE__, __TIME__);
    system_info->level = diagnostic_msgs::DiagnosticStatus::OK;
    system_info->name = "System";
    system_info->message = system_status_msg;
    system_info->hardware_id = "MCIU";
    // For each ODrive add its diagnostic message


//    for (ODriveS1* odrive : odrives) {
//        odrive->init();
//    }

    log_msg = "Advertising diagnostics publishers";
    node_handle.loginfo(log_msg.c_str());

    node_handle.advertise(sys_diag_pub);
    sys_diag_pub.publish(&system_diagnostics);

    log_msg = "Attempting to preform first spin";
    node_handle.loginfo(log_msg.c_str());
    node_handle.spinOnce();
    log_msg = "Setup complete";
    node_handle.loginfo(log_msg.c_str());
}

void loop() {

    uint32_t loop_start = micros(); // Get the time at the start of the loop
    digitalWriteFast(LED_BUILTIN, LOW); // Turn on the LED

    system_message_count = 0;
    for (char *string: system_status_messages) {
        string[0] = '\0';
    }
    system_info->level = diagnostic_msgs::DiagnosticStatus::OK;

    // Get the teensy temperature

    sprintf(system_info_strings[0], "%.1fC", tempmonGetTemp());
    check_temp();

    for (ODriveS1 *odrive: odrives) {
        if (odrive == nullptr) continue;
        odrive->refresh_data();
    }

    for (ActuatorsROS *actuator: actuators_ros) {
        if (actuator == nullptr) continue;
        actuator->update();
    }

    if (node_handle.connected()) {
        for (ODrive_ROS *odrive: odrive_ros) {
            if (odrive == nullptr) {
                continue;
            }
            odrive->publish_all();
        }
    }

    system_diagnostics.header.stamp = node_handle.now();
    system_diagnostics.header.seq++;

    String log_msg = "";
    int8_t spin_result = 0;
    if (!node_handle.connected()) {
//        node_handle.logerror("NodeHandle not properly configured");
    }
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

    digitalWriteFast(LED_BUILTIN, HIGH); // Turn off the LED
    // Allow the actuator bus to preform serial communication for the remaining time in the loop
    while (actuator_bus.spin(micros() - loop_start > 50000)) {
        yield();  // Yield to other tasks
    }

    uint32_t remaining_memory = freeram();
    if (remaining_memory < 100000) {
        system_info->level = diagnostic_msgs::DiagnosticStatus::WARN;
        sprintf(system_status_messages[system_message_count++], "Low Memory");
        if (remaining_memory < 30000) {
            system_info->level = diagnostic_msgs::DiagnosticStatus::ERROR;
            sprintf(system_status_messages[system_message_count++], "Out of Memory");
        }
    }

    sprintf(system_info_strings[1], "%.2f Mhz (%.2f%%)", F_CPU_ACTUAL / 1000000.0,
            100.0 * F_CPU_ACTUAL / F_CPU);
    sprintf(system_info_strings[2], "%luus (%.2fHz)", micros() - loop_start, 1000000.0 / (micros() - loop_start));
    sprintf(system_info_strings[3], "%.2fKib (%.2f%%)", remaining_memory / 1024.0,
            100.0 * remaining_memory / 512000.0);
    sprintf(system_info_strings[4], "%lu", can1.getTXQueueCount());
    sprintf(system_info_strings[5], "%lu", can1.getRXQueueCount());
    sprintf(system_info_strings[6], "%lums", actuator_bus.round_trip_time());

    uint32_t loop_time = micros() - loop_start;
    if (loop_time > 50000) {
        system_info->level = diagnostic_msgs::DiagnosticStatus::WARN;
        sprintf(system_status_messages[system_message_count++], "Slow Loop");
        // Set the CPU to the overclocked speed
#ifdef ENABLE_BOOST
        if (tempmonGetTemp() < WARN_TEMP) {
            cpu_freq = CPU_FREQ_BOOST;
            cpu_boost_time = micros();
        }
    } else if (micros() - cpu_boost_time > UN_BOOST_TIME) {
        cpu_freq = CPU_FREQ_BASE;
    }
#else
    }
#endif

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