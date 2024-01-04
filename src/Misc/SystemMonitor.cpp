//
// Created by Jay on 12/21/2023.
//

#include "SystemMonitor.h"

extern unsigned long _heap_start;  // start of heap
extern unsigned long _heap_end;    // end of heap
extern char          *__brkval;    // current top of heap

#define WARN_TEMP 65.0 // Degrees C
#define FAULT_TEMP 75.0 // Degrees C


void SystemMonitor::update() {
    uint32_t free = (char *)&_heap_end - __brkval;
    output_array.fields.mciu_temp =   (int32_t) (tempmonGetTemp() * 100);
    output_array.fields.mciu_heap =   (int32_t) (free);
    output_array.fields.mciu_uptime = (int32_t) (millis());
}

EStopDevice::TRIP_LEVEL SystemMonitor::tripped(char *tripped_device_name, char *tripped_device_message) {
    sprintf(tripped_device_name, "System Monitor");
    char temp[100];
    TRIP_LEVEL trip_level = EStopDevice::TRIP_LEVEL::NO_FAULT;
    if (tempmonGetTemp() > WARN_TEMP) {
        sprintf(temp, "MCIU TEMP HIGH: %fC-", tempmonGetTemp());
        trip_level = EStopDevice::TRIP_LEVEL::WARNING;
        strlcat(tripped_device_message, temp, 100);
    }
    if (output_array.fields.mciu_heap < 40000) {
        sprintf(temp, "MCIU HEAP LOW: %ld-", output_array.fields.mciu_heap);
        trip_level = EStopDevice::TRIP_LEVEL::WARNING;
        strlcat(tripped_device_message, temp, 100);
    }
    if (tempmonGetTemp() > FAULT_TEMP)
        trip_level = EStopDevice::TRIP_LEVEL::FAULT;
    if (output_array.fields.mciu_heap < 20000)
        trip_level = EStopDevice::TRIP_LEVEL::FAULT;

    if (output_array.fields.mciu_utilization > 10500) {
        sprintf(temp, "MCIU TIMING CONSTRAINTS VIOLATED: %02.2f%%-", output_array.fields.mciu_utilization / 100.0f);
        trip_level = EStopDevice::TRIP_LEVEL::FAULT;
        strlcat(tripped_device_message, temp, 100);
    } else if (output_array.fields.mciu_utilization > 8500) {
        sprintf(temp, "MCIU HIGH LOAD: %02.2f%%-", output_array.fields.mciu_utilization / 100.0f);
        trip_level = EStopDevice::TRIP_LEVEL::WARNING;
        strlcat(tripped_device_message, temp, 100);
    }


    // Remove trailing dash
    if (trip_level != EStopDevice::TRIP_LEVEL::NO_FAULT)
        tripped_device_message[strlen(tripped_device_message) - 1] = '\0';
    return trip_level;
}
