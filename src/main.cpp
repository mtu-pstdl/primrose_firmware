
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <ros.h>
#include <std_srvs/SetBool.h>
#include "../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros.h"
#include "ODrive/ODriveS1.h"

ros::NodeHandle node_handle;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> can1;

ODriveS1* odrives[6];


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

//    node_handle.initNode();

    Serial.begin(115200); // 115kbps

    // Set up the CAN bus
    can1.begin();
    can1.setBaudRate(500000); // 500kbps


    // Set up the ODrives
//    odrives[0] = new ODriveS1(0, "FLD", &can1);
//    odrives[1] = new ODriveS1(1, "FRD", &can1);
//    odrives[2] = new ODriveS1(2, "BLD", &can1);
//    odrives[3] = new ODriveS1(3, "BRD", &can1);
//    odrives[4] = new ODriveS1(4, "TRENCH", &can1);

    odrives[0] = new ODriveS1(0, "FLD", &can1);
    odrives[1] = new ODriveS1(1, "FRD", &can1);
    odrives[2] = new ODriveS1(2, "BLD", &can1);
    odrives[3] = new ODriveS1(3, "BRD", &can1);
    odrives[4] = new ODriveS1(4, "TRENCH", &can1);
    odrives[5] = new ODriveS1(5, "CONVEY", &can1);


    // Set MailBox 0 to receive all messages
    can1.setMBFilter(MB0, 0x000, 0x7FF);
    // Setup a callback for MB 0
    can1.onReceive(MB0, can_event);


}

void loop() {

    // Refresh the data from the ODrives
    for (ODriveS1* odrive : odrives) {
        odrive->refresh_data();
    }

    Serial.println("Dumping data");
    for (ODriveS1* odrive: odrives){
        auto* state = odrive->get_state_string();
        Serial.println(state->c_str());
        free(state);
    }

//    uint8_t result = node_handle.spinOnce();
//
//    if (result != ros::SPIN_OK) {
//
//    }

}