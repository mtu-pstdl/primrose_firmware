//
// Created by Jay on 3/8/2024.
//

#ifndef PRIMROSE_MCIU_HIGHSPEEDLOGGER_H
#define PRIMROSE_MCIU_HIGHSPEEDLOGGER_H


#include "ROSNode.h"
#include "ODrive/odrive_constants.h"
#include "ODrive/ODrivePro.h"

class HighSpeedLogger : public ROSNode {

private:

    ODrivePro* odrive = nullptr;
    std_msgs::Int32MultiArray* output_topic;

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

public:
    explicit HighSpeedLogger(std_msgs::Int32MultiArray* output_topic){
        this->output_topic = output_topic;
    }

    void attach_to_odrive(ODrivePro* target_odrive);

    void detach_from_odrive();


    static void log_callback(HighSpeedLogger* logger, odrive::command_ids command_id);

    LogData get_log_data(ODrivePro *target, odrive::command_ids data_type);

    uint64_t get_micros_64();

    void add_log(LogData log_data);
};


#endif //PRIMROSE_MCIU_HIGHSPEEDLOGGER_H
