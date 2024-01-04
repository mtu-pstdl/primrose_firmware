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
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h"

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

    union OutputArray {
        struct {
            int32_t mciu_temp;         // The temperature of the MCU in degrees C (0.01 C resolution)
            int32_t mciu_heap;         // The remaining heap of the MCU in bytes
            int32_t mciu_uptime;       // The uptime of the MCU in milliseconds
            int32_t mciu_utilization;  // The utilization of the MCU in percent (0-100)
            int32_t mciu_freq;         // The frequency of main loop in Hz (0.01 Hz resolution)
            int32_t adau_temp;
            int32_t adau_heap;
            int32_t adau_uptime;
            int32_t unused;
        } fields;
        int32_t data[10];
    } output_array = {};

    std_msgs::Int32MultiArray* system_monitor_topic = nullptr;

public:

    explicit SystemMonitor(std_msgs::Int32MultiArray* system_monitor_msg){
        this->system_monitor_topic = system_monitor_msg;
        this->adau_info = new ADAU_Sensor(0x00, &this->adau_data, sizeof(this->adau_data));
        this->system_monitor_topic->data_length = 10;
        this->system_monitor_topic->data = output_array.data;
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

    /**
     * @brief Update the loop info for the system monitor
     * @param execution_time The time the main loop took before arriving at the delay (us)
     * @param loop_time      The time the whole loop took including the delay (us)
     */
    void update_loop_info(uint32_t execution_time, uint32_t loop_time) {
        // Utilization is the percentage of time the loop that was within the 50000 us limit and has no relation
        // loop time, the scale is .01% per unit
        output_array.fields.mciu_utilization = (int32_t) (execution_time * 10000 / 50000);

        // Frequency is the number of times the loop ran in a second (0.01 Hz resolution)
        output_array.fields.mciu_freq = (int32_t) (100000000 / loop_time);

    }

    void update() override;

    void publish() override {}

    EStopDevice::TRIP_LEVEL tripped(char* tripped_device_name, char* tripped_device_message) override;

};


#endif //PRIMROSE_MCIU_SYSTEMMONITOR_H
