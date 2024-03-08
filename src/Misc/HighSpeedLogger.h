//
// Created by Jay on 3/8/2024.
//

#ifndef PRIMROSE_MCIU_HIGHSPEEDLOGGER_H
#define PRIMROSE_MCIU_HIGHSPEEDLOGGER_H


#include "ROSNode.h"
#include "ODrive/odrive_constants.h"
#include "ODrive/ODrivePro.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/std_msgs/UInt32MultiArray.h"
#include "../../.pio/libdeps/teensy40/Rosserial Arduino Library/src/ros/publisher.h"


class HighSpeedLogger : public ROSNode {

private:

    ODrivePro* odrive = nullptr;

    volatile boolean halt_logging = false;  // Set high when the buffer is being accessed

    struct LogData {
        uint8_t  command_id: 8;       // odrive::command_ids
        uint32_t timestamp_high: 24;
        uint32_t timestamp_low : 32;
        union {
            float_t float_data;
            int32_t int_data;
        } data_1;
        union {
            float_t float_data;
            int32_t int_data;
        } data_2;
    };

    /**
     * Stores the data logged before it is packed and sent to the output topic
     */
    struct LogDataMessage {
        LogData log_data[56] = {};
        uint8_t total_data_points = 0;
    };

    struct LogDataBuffer {
        LogDataMessage log_data_messages[6] = {};
        uint8_t current_message = 0;
    } log_data_buffer;

    std_msgs::UInt32MultiArray* output_message;
    uint32_t output_message_data[sizeof(LogDataMessage) / sizeof(uint32_t)];
    ros::Publisher* publisher;

public:

    HighSpeedLogger(std_msgs::UInt32MultiArray* output_message, ros::Publisher* output_topic){
        this->output_message = output_message;
        this->publisher = output_topic;
        this->output_message->data = this->output_message_data;
        this->output_message->data_length = 0;

    }

    void attach_to_odrive(ODrivePro* target_odrive);

    void detach_from_odrive();


    static void log_callback(HighSpeedLogger* logger, odrive::command_ids command_id);

    LogData get_log_data(ODrivePro *target, odrive::command_ids data_type);

    uint64_t get_micros_64();

    void add_log(LogData log_data);

    void update() override;

    void write_to_output(LogDataMessage *message);
};


#endif //PRIMROSE_MCIU_HIGHSPEEDLOGGER_H
