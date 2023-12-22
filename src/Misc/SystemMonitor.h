//
// Created by Jay on 12/21/2023.
//

#ifndef PRIMROSE_MCIU_SYSTEMMONITOR_H
#define PRIMROSE_MCIU_SYSTEMMONITOR_H

#include "ROSNode.h"
#include "EStopDevice.h"
#include "ADAU_Interfaces/ADAU_Sensor.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/DiagnosticStatus.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/diagnostic_msgs/KeyValue.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/UInt32MultiArray.h"

/**
 * The system monitor reports the health of both the MCIU and the ADAU
 */
class SystemMonitor : public ROSNode, public EStopDevice {

private:

    ros::NodeHandle* node_handle = nullptr;

    ADAU_Sensor* adau_info = nullptr;

#pragma pack(push, 1)
    struct adau_data_struct {
        float_t  system_temp;
        uint32_t system_uptime;
        uint32_t remaining_memory;
    } adau_data{};
#pragma pack(pop)

    std_msgs::UInt32MultiArray* system_monitor_topic = nullptr;

public:

    explicit SystemMonitor(std_msgs::UInt32MultiArray* system_monitor_msg){
        this->system_monitor_topic = system_monitor_msg;
        this->adau_info = new ADAU_Sensor(0x00, &this->adau_data, sizeof(this->adau_data));
        this->system_monitor_topic->data_length = 10;
        this->system_monitor_topic->data = new uint32_t[10];
        for (int i = 0; i < 10; i++) {
            this->system_monitor_topic->data[i] = 0;
        }
        /**
         * Data format:
         * 0: MCIU Temperature
         * 1: MCIU Remaining Heap
         * 2: MCIU Uptime
         * 3: MCIU Utilization
         * 4: ADAU Temperature
         * 5: ADAU Remaining Heap
         * 6: ADAU Uptime
         * 7: Unused
         */
    }

    void subscribe(ros::NodeHandle* nh) override {
        this->node_handle = nh;
        this->node_handle->loginfo("System Monitor initialized successfully");
    }

    void update() override;

    void publish() override {}

    boolean tripped(char* tripped_device_name, char* tripped_device_message) override;

};


#endif //PRIMROSE_MCIU_SYSTEMMONITOR_H
