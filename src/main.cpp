
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

ros::NodeHandle node_handle;
FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> can1;

//#define HIGH_SPEED_USB

ODriveS1* odrives[6];
ODrive_ROS* odrive_ros[6];

ActuatorUnit* actuators[4];
ActuatorsROS* actuators_ros[4];
Actuators actuator_bus;

// Setup global publishers
//diagnostic_msgs::DiagnosticArray odrive_diagnostics;
//diagnostic_msgs::DiagnosticArray actuator_diagnostics;
diagnostic_msgs::DiagnosticArray system_diagnostics;

diagnostic_msgs::DiagnosticStatus* system_info;

//ros::Publisher odrive_diag_pub("/diagnostics_motor", &system_diagnostics);
//ros::Publisher actuator_diag_pub("/diagnostics_actuator", &actuator_diagnostics);
ros::Publisher sys_diag_pub("/diagnostics", &system_diagnostics);

uint32_t last_print = 0;

bool startup_info_print_once = false;

void can_event(const CAN_message_t &msg) {
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

//String* can_debug(){
//    auto *debug = new String();
//    // Print info about state of CAN bus
//    debug->concat("CAN1: ");
//    debug->concat(can1.isBusOff() ? "OFF" : "ON");

//}

void setup() {

//    Serial.begin(9600); // 115kbps
//    Serial.begin(115200); // 115kbps
//    Serial.println("Starting up");
//    Serial1.begin(38400); // 38.4kbps

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
    can1.onReceive(can_event);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();


    log_msg = "CAN bus initialised";
    node_handle.loginfo(log_msg.c_str());


    log_msg = "Initialising ODriveS1 objects";
    node_handle.loginfo(log_msg.c_str());

    odrives[0] = new ODriveS1(0, new String("00"), &can1);
    odrives[1] = new ODriveS1(1, new String("01"), &can1);
    odrives[2] = new ODriveS1(2, new String("02"), &can1);
    odrives[3] = new ODriveS1(3, new String("03"), &can1);
    odrives[4] = new ODriveS1(4, new String("04"), &can1);
    odrives[5] = new ODriveS1(5, new String("05"), &can1);

    // Setup the diagnostics array
//    odrive_diagnostics.status_length = 6;
//    odrive_diagnostics.status = new diagnostic_msgs::DiagnosticStatus[6];
//
//    actuator_diagnostics.status_length = 4;
//    actuator_diagnostics.status = new diagnostic_msgs::DiagnosticStatus[4];

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

//

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

//    delay(1000);


    for (ODrive_ROS* odrive : odrive_ros) {
        if (odrive == nullptr) continue;
        log_msg = "Advertising ODrive";
        node_handle.loginfo(log_msg.c_str());
//        odrive->advertise_subscribe(&node_handle);
    }


    log_msg = "Setting up system diagnostics";
    node_handle.loginfo(log_msg.c_str());

    system_info = &system_diagnostics.status[10];
    system_info->values_length = 1;
    system_info->values = new diagnostic_msgs::KeyValue[1];
    system_info->values[0].key = "Temperature";
    system_info->values[0].value = "0";
    system_info->level = diagnostic_msgs::DiagnosticStatus::OK;
    system_info->name = "System";
    system_info->message = "System is running";
    system_info->hardware_id = "MCIU";
    // For each ODrive add its diagnostic message


//    for (int i = 0; i < 4; i++) {
//        if (actuators[i] == nullptr) {
//            actuators_ros[i] = nullptr;
//            continue;
//        }
//        actuators_ros[i] = new ActuatorsROS(actuators[i], &node_handle);
//    }

//    for (ActuatorsROS* actuator : actuators_ros) {
//        if (actuator == nullptr) {
//            continue;
//        }
//        actuator->advertise_subscribe(&node_handle);
//    }

//    for (ODriveS1* odrive : odrives) {
//        odrive->init();
//    }

    log_msg = "Advertising diagnostics publishers";
    node_handle.loginfo(log_msg.c_str());

//    node_handle.advertise(odrive_diag_pub);
//    node_handle.advertise(actuator_diag_pub);
    node_handle.advertise(sys_diag_pub);

//    odrive_diag_pub.publish(&odrive_diagnostics);
//    actuator_diag_pub.publish(&actuator_diagnostics);
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

    // Get the teensy temperature
//    diagnostic_array.status[10].values[0].value = String(tempmonGetTemp()).c_str();

    for (ODriveS1* odrive : odrives) {
        if (odrive == nullptr) continue;
        odrive->refresh_data();
    }

    for (ActuatorsROS* actuator : actuators_ros) {
        if (actuator == nullptr) continue;
        actuator->update();
    }

    if (node_handle.connected()){
        for (ODrive_ROS* odrive : odrive_ros) {
            if (odrive == nullptr) {
                continue;
            }
            odrive->publish_all();
        }
    }

//    odrive_diagnostics.header.stamp = node_handle.now();
//    actuator_diagnostics.header.stamp = node_handle.now();
    system_diagnostics.header.stamp = node_handle.now();

//    odrive_diagnostics.header.seq++;
//    actuator_diagnostics.header.seq++;
    system_diagnostics.header.seq++;

    // Publish the diagnostics
//    odrive_diag_pub.publish(&odrive_diagnostics);
//    actuator_diag_pub.publish(&actuator_diagnostics);
    sys_diag_pub.publish(&system_diagnostics);

    int8_t spin_result = 0;
    String log_msg = "Loop time: " + String((micros() - loop_start) / 1000) + "ms";
    node_handle.loginfo(log_msg.c_str());
    if (!node_handle.connected()){
//        node_handle.logerror("NodeHandle not properly configured");
    }
    spin_result = node_handle.spinOnce(); // 50ms timeout

    log_msg = "Spin time: " + String(micros() - loop_start);
//    node_handle.loginfo(log_msg.c_str());

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
    // Delay for the remaining time in the loop
    if (50000 - (micros() - loop_start) < 0) {
        log_msg = "Loop time exceeded 50ms by: " + String(micros() - loop_start - 50000);
        node_handle.logwarn(log_msg.c_str());
    } else {
//        delayMicroseconds(50000 - (micros() - loop_start));
    }
//    delayMicroseconds(50000);
}