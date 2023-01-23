
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <ros.h>
#include <std_srvs/SetBool.h>
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include "ODrive/ODriveS1.h"
#include "ODrive/ODrive_ROS.h"
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Float32.h"

ros::NodeHandle node_handle;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> can1;

ODriveS1* odrives[6];
ODrive_ROS* odrive_ros[6];

// Setup global publishers
std_msgs::Float32 system_temperature;
ros::Publisher system_temperature_pub("/mciu/system_temperature", &system_temperature);

bool startup_info_print_once = false;


void can_event(const CAN_message_t &msg) {
    // Check node ID (Upper 6 bits of CAN ID)
    uint8_t node_id = msg.id >> 5;
    for (ODriveS1* odrive : odrives) {
        if (odrive->get_can_id() == node_id) {
            odrive->on_message(msg);
        }
    }
}


void setup() {

//    Serial.begin(9600); // 115kbps
//    Serial1.begin(38400); // 38.4kbps

    node_handle.getHardware()->setBaud(115200);
    node_handle.initNode();
//    node_handle.spinOnce();

    node_handle.advertise(system_temperature_pub);

    String log_msg = "Initialising CAN bus at 500kbps";
    node_handle.loginfo(log_msg.c_str());

    // Set up the CAN bus
//    can1.begin();
//    can1.setBaudRate(500000); // 500kbps

    log_msg = "CAN bus initialised";
    node_handle.loginfo(log_msg.c_str());

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    odrives[0] = new ODriveS1(0, new String("00"), &can1);
    odrive_ros[0] = new ODrive_ROS(odrives[0], reinterpret_cast<const char *>('A'));
    odrives[1] = new ODriveS1(1, new String("01"), &can1);
    odrive_ros[1] = new ODrive_ROS(odrives[1], reinterpret_cast<const char *>('B'));
    odrives[2] = new ODriveS1(2, new String("02"), &can1);
    odrive_ros[2] = new ODrive_ROS(odrives[2], reinterpret_cast<const char *>('C'));
    odrives[3] = new ODriveS1(3, new String("03"), &can1);
    odrive_ros[3] = new ODrive_ROS(odrives[3], reinterpret_cast<const char *>('D'));
    odrives[4] = new ODriveS1(4, new String("04"), &can1);
    odrive_ros[4] = new ODrive_ROS(odrives[4], reinterpret_cast<const char *>('E'));
    odrives[5] = new ODriveS1(5, new String("05"), &can1);
    odrive_ros[5] = new ODrive_ROS(odrives[5], reinterpret_cast<const char *>('F'));


    // Make sure there are no duplicate ODrive nodes

    for (ODrive_ROS* odrive : odrive_ros) {
        if (odrive == nullptr) {
            continue;
        }
        odrive->advertise(&node_handle);
        node_handle.spinOnce();
    }


//
////     Set MailBox 0 to receive all messages
//    can1.setMBFilter(MB0, 0x000, 0x7FF);
////     Setup a callback for MB 0
//    can1.onReceive(MB0, can_event);

}

void loop() {

    uint32_t loop_start = micros(); // Get the time at the start of the loop
    digitalWriteFast(LED_BUILTIN, HIGH); // Turn on the LED

    if (!startup_info_print_once) {
        node_handle.loginfo("Startup complete");
        for (ODrive_ROS* odrive : odrive_ros) {
            if (odrive == nullptr) {
                continue;
            }
//            node_handle.loginfo(odrive->get_odrive()->*name.c_str());
        }
        startup_info_print_once = true;
    }

    uint8_t result = node_handle.spinOnce();

    if (result != ros::SPIN_OK) {

    }

    digitalWriteFast(LED_BUILTIN, LOW); // Turn off the LED
    // Wait until the loop has been running for 50ms
    while (micros() - loop_start < 50000) {
        delayMicroseconds(10);
    }

}