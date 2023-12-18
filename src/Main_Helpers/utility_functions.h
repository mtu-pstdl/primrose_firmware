//
// Created by Jay on 12/18/2023.
//

#ifndef PRIMROSE_MCIU_UTILITY_FUNCTIONS_H
#define PRIMROSE_MCIU_UTILITY_FUNCTIONS_H

#include <cstdint>
#include <Arduino.h>
#include <EEPROM.h>

#define WATCHDOG_FLAG_ADDR 4282 // The address in eeprom where if a watchdog was triggered it will be set
#define RESTART_ADDR 0xE000ED0C  // Application Restart and Control Register
extern unsigned long _heap_start;  // start of heap
extern unsigned long _heap_end;  // end of heap
extern char          *__brkval;  // current top of heap

#define CPU_FREQ_BASE 300000000
#define CPU_FREQ_MIN 24000000 // 24 MHz
#define WARN_TEMP 65.0 // Degrees C
#define THROTTLE_TEMP 75.0 // Degrees C
uint32_t cpu_freq = CPU_FREQ_BASE;

#if defined(__IMXRT1062__)
extern "C" uint32_t set_arm_clock(uint32_t frequency);
#endif

extern WDT_T4<WDT1> wdt;

uint32_t last_ram_time = 0;
int last_ram = 0;
int last_ram_usage = 0;

void watchdog_violation() {
    // If the watchdog timer is triggered then reset the system
    // This will only happen if the loop takes longer than MAX_LOOP_TIME
    // This is a hard limit
    wdt.reset();
    EEPROM.write(WATCHDOG_FLAG_ADDR, 0x5A); // Set the watchdog flag in eeprom
    // write to the restart register (Application Restart and Control Register) to hard reset the system
    *(volatile uint32_t *)RESTART_ADDR = 0x5FA0004;
}

int freeram() {
    uint32_t free = (char *)&_heap_end - __brkval;
    // If free is less than 1000 then we restart the system and set the memory exhaustion flag
    if (free < 1000) {
        EEPROM.write(WATCHDOG_FLAG_ADDR, 0x5B);
        *(volatile uint32_t *)RESTART_ADDR = 0x5FA0004;
    }
    return free;
}

void check_temp(){
    if (tempmonGetTemp() > WARN_TEMP) {
        if (tempmonGetTemp() > THROTTLE_TEMP) {
            set_arm_clock(CPU_FREQ_MIN);
        }
    } else set_arm_clock(cpu_freq);  // 600 MHz (default)
}

int ram_usage_rate(){
    int free_ram = freeram();
    int ram_usage = last_ram - free_ram;
    last_ram = free_ram;
    if (millis() - last_ram_time > 1000 || ram_usage > 0) {
        last_ram_time = millis();
        last_ram_usage = ram_usage;
    }
    return last_ram_usage;
}

#endif //PRIMROSE_MCIU_UTILITY_FUNCTIONS_H
