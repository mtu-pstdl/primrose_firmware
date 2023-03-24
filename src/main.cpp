
#include <Arduino.h>
#include <Actuators/Actuators.h>
#include <FlexCAN_T4.h>
//#include <ros.h>
//#include <std_srvs/SetBool.h>
//#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include "ODrive/ODriveS1.h"
//#include "ODrive/ODrive_ROS.h"
//#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Float32.h"
#include "Actuators/ActuatorUnit.h"

//ros::NodeHandle node_handle;
FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_64> can1;

//#define HIGH_SPEED_USB

ODriveS1* odrives[6];
//ODrive_ROS* odrive_ros[6];

ActuatorUnit* actuators[4];
//ActuatorsROS* actuators_ros[4];
Actuators actuator_bus;

// Setup global publishers
//std_msgs::Float32 system_temperature;
//diagnostic_msgs::DiagnosticStatus system_info;
//ros::Publisher system_temperature_pub("/mciu/system_temperature", &system_temperature);

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
    Serial.begin(115200); // 115kbps
    Serial.println("Starting up");
//    Serial1.begin(38400); // 38.4kbps

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(1000);

#ifdef HIGH_SPEED_USB
//    node_handle.getHardware()->setBaud(500000); // 500kbps
#else
//    node_handle.getHardware()->setBaud(115200); // 115kbps
#endif
//    node_handle.setSpinTimeout(50); // 50ms
//    node_handle.initNode();
//    node_handle.spinOnce();

//    node_handle.advertise(system_temperature_pub);

//    String log_msg = "Initialising CAN bus at 500kbps";
//    node_handle.loginfo(log_msg.c_str());

    // Set up the CAN bus
    can1.begin();
    can1.setBaudRate(500000); // 500kbps
//    can1.setMaxMB(64);  // 64 message buffers
    can1.onReceive(can_event);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();


//    log_msg = "CAN bus initialised";
//    node_handle.loginfo(log_msg.c_str());


    odrives[0] = new ODriveS1(0, new String("00"), &can1);
//    odrives[1] = new ODriveS1(1, new String("01"), &can1);
//    odrives[2] = new ODriveS1(2, new String("02"), &can1);
//    odrives[3] = new ODriveS1(3, new String("03"), &can1);
//    odrives[4] = new ODriveS1(4, new String("04"), &can1);
//    odrives[5] = new ODriveS1(5, new String("05"), &can1);
//
//    actuators[0] = new ActuatorUnit(&actuator_bus, 0x80);
//    actuators[1] = new ActuatorUnit(&actuator_bus, 0x81);
//    actuators[2] = new ActuatorUnit(&actuator_bus, 0x82);
//    actuators[3] = new ActuatorUnit(&actuator_bus, 0x83);

    // Make sure there are no duplicate ODrive nodes

//    for (ODrive_ROS* odrive : odrive_ros) {
//        if (odrive == nullptr) {
//            continue;
//        }
//        odrive->advertise(&node_handle);
//    }

//    for (ODriveS1* odrive : odrives) {
//        odrive->init();
//    }

    startup_info_print_once = false;

    Serial.printf("Setup complete (%dms)\n", millis());
    Serial.println("Starting loop");
}

void loop() {

    uint32_t loop_start = micros(); // Get the time at the start of the loop
    digitalWriteFast(LED_BUILTIN, LOW); // Turn on the LED

    // Get the teensy temperature
//    system_temperature.data = (float) tempmonGetTemp();
//    system_temperature_pub.publish(&system_temperature);

    for (ODriveS1* odrive : odrives) {
        if (odrive == nullptr) continue;
        odrive->refresh_data();
    }

    for (ActuatorUnit* actuator : actuators) {
        if (actuator == nullptr) continue;
        actuator->update();
    }

//    for (ODrive_ROS* odrive : odrive_ros) {
//        if (odrive == nullptr) {
//            continue;
//        }
//        odrive->publish_all();
//    }

//    if (!node_handle.connected()){
//        node_handle.logerror("NodeHandle not properly configured");
//    }

//    int8_t spin_result = node_handle.spinOnce(); // 50ms timeout
//
//    switch (spin_result) {
//        case ros::SPIN_OK:
//            break;
//        case ros::SPIN_ERR:
//            node_handle.logerror("Spin error");
//            // Collect info about the error
//
//            break;
//        case ros::SPIN_TIMEOUT:
//            node_handle.logwarn("Spin timeout");
//            break;
//        default:
//            String log_msg = "Unknown spin result: " + String(spin_result);
//            node_handle.logerror(log_msg.c_str());
//            break;
//    }

    // Print to serial the state of all devices once per 5 seconds
    if (millis() - last_print > 5000) {
        Serial.println("Printing status");
        last_print = millis();
        Serial.clear();
        for (ODriveS1* odrive : odrives) {
            if (odrive == nullptr) continue;

            auto* data = odrive->get_state_string();
            if (data != nullptr) {
                Serial.println(*data);
            }
            delete data;
        }
//        for (ActuatorUnit* actuator : actuators) {
//            auto* data = actuator->get_status_string();
//            if (data != nullptr) {
//                Serial.print(*data);
//            }
//            delete data;
//        }
//        auto* data = actuator_bus.get_status_string();
//        if (data != nullptr) {
//            Serial.print(*data);
//        }
//        delete data;
        String sys_info = "--- Sys Info ---\r\n";
        sys_info += "CAN bus: " + String(can1.getRXQueueCount()) + " RX messages in queue\r\n";
        sys_info += "CAN bus: " + String(can1.getTXQueueCount()) + " TX messages in queue\r\n";
        sys_info += "Temperature: " + String(tempmonGetTemp()) + "\r\n";
        Serial.println(sys_info);
    }

    digitalWriteFast(LED_BUILTIN, HIGH); // Turn off the LED
    // Allow the actuator bus to preform serial communication for the remaining time in the loop
    while (actuator_bus.spin(micros() - loop_start > 50000)) {
        yield();  // Yield to other tasks
    }
    delay(100);
}