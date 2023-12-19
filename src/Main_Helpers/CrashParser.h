//
// Created by Jay on 12/18/2023.
//

#ifndef PRIMROSE_MCIU_CRASHPARSER_H
#define PRIMROSE_MCIU_CRASHPARSER_H

#include <cstdint>

class CrashParser {

    struct crash_dump_line {
        char line[100] = {};
        crash_dump_line *next{};
    };

    crash_dump_line *output = new crash_dump_line;
    crash_dump_line *current_line = output;

    struct fault_info_struct {
        uint32_t len;   // all fields must be 32 bits
        uint32_t ipsr;  // interrupt program status register
        uint32_t cfsr;  // configurable fault status register
        uint32_t hfsr;  // hard fault status register
        uint32_t mmfar; // memory management fault address register
        uint32_t bfar;  // bus fault address register
        uint32_t ret;   // return address
        uint32_t xpsr;  // exception program status register
        float  temp;    // temperature
        uint32_t time;  // time of crash
        uint32_t crc;  // crc must be last
    };

    struct fault_info_struct *crash_info = (struct fault_info_struct *)0x2027FF80;

    boolean csfr_to_string(fault_info_struct *info){
        if (info->cfsr == 0) return false;
        if ((info->cfsr & 1) == 1) {
            sprintf(current_line->line, "FAULT_TYPE: (IACCVIOL) Instruction Access Violation");
            next_line();
        } else  if (((info->cfsr & (0x02)) >> 1) == 1) {
            sprintf(current_line->line, "FAULT_TYPE: (DACCVIOL) Data Access Violation");
            next_line();
        } else if (((info->cfsr & (0x08)) >> 3) == 1) {
            sprintf(current_line->line, "FAULT_TYPE: (MUNSTKERR) MemMange Fault on Unstacking");
            next_line();
        } else if (((info->cfsr & (0x10)) >> 4) == 1) {
            sprintf(current_line->line, "FAULT_TYPE: (MSTKERR) MemMange Fault on stacking");
            next_line();
        } else if (((info->cfsr & (0x20)) >> 5) == 1) {
            sprintf(current_line->line, "FAULT_TYPE: (MLSPERR) MemMange Fault on FP Lazy State");
            next_line();
        }
        if (((info->cfsr & (0x80)) >> 7) == 1) {
            if (info->mmfar < 32) {
                sprintf(current_line->line, "(MMARVALID) Accessed Address: 0x%08lX (nullptr)", info->mmfar);
            } else {
                sprintf(current_line->line, "(MMARVALID) Accessed Address: 0x%08lX", info->mmfar);
            }
            next_line();
        }
        if (((info->cfsr & 0x100) >> 8) == 1) {
            sprintf(current_line->line, "(IBUSERR) Instruction Bus Error");
            next_line();
        } else  if (((info->cfsr & (0x200)) >> 9) == 1) {
            sprintf(current_line->line, "(PRECISERR) Data bus error(address in BFAR)");
            next_line();
        } else if (((info->cfsr & (0x400)) >> 10) == 1) {
            sprintf(current_line->line, "(IMPRECISERR) Data bus error but address not related to instruction");
            next_line();
        } else if (((info->cfsr & (0x800)) >> 11) == 1) {
            sprintf(current_line->line, "(UNSTKERR) Bus Fault on unstacking for a return from exception");
            next_line();
        } else if (((info->cfsr & (0x1000)) >> 12) == 1) {
            sprintf(current_line->line, "(STKERR) Bus Fault on stacking for exception entry");
            next_line();
        } else if (((info->cfsr & (0x2000)) >> 13) == 1) {
            sprintf(current_line->line, "(LSPERR) Bus Fault on FP lazy state preservation");
            next_line();
        }
        if (((info->cfsr & (0x8000)) >> 15) == 1) {
            sprintf(current_line->line, "(BFARVALID) Accessed Address: 0x%08lX", info->bfar);
            next_line();
        }
        if (((info->cfsr & 0x10000) >> 16) == 1) {
            sprintf(current_line->line, "(UNDEFINSTR) Undefined instruction");
            next_line();
        } else  if (((info->cfsr & (0x20000)) >> 17) == 1) {
            sprintf(current_line->line, "(INVSTATE) Instruction makes illegal use of EPSR");
            next_line();
        } else if (((info->cfsr & (0x40000)) >> 18) == 1) {
            sprintf(current_line->line, "(INVPC) Usage fault: invalid EXC_RETURN");
            next_line();
        } else if (((info->cfsr & (0x80000)) >> 19) == 1) {
            sprintf(current_line->line, "(NOCP) No Coprocessor");
            next_line();
        } else if (((info->cfsr & (0x1000000)) >> 24) == 1) {
            sprintf(current_line->line, "(UNALIGNED) Unaligned access UsageFault");
            next_line();
        } else if (((info->cfsr & (0x2000000)) >> 25) == 1) {
            sprintf(current_line->line, "(DIVBYZERO) Divide by zero");
            next_line();
        }
        return true;
    }

    bool isvalid(const struct fault_info_struct *info) {
        uint32_t i, crc;
        const uint32_t *data, *end;

        if (info->len != sizeof(*info) / 4) return false;
        data = (uint32_t *)info;
        end = data + (sizeof(*info) / 4 - 1);
        crc = 0xFFFFFFFF;
        while (data < end) {
            crc ^= *data++;
            for (i=0; i < 32; i++) crc = (crc >> 1) ^ (crc & 1)*0xEDB88320;
        }
        if (crc != info->crc) return false;
        return true;
    }

    void next_line() {
        if (output == nullptr) {
            output = new crash_dump_line;
            current_line = output;
            output->next = nullptr;
            sprintf(output->line, "----- END OF CRASH DUMP -----");
        } else {
            current_line->next = new crash_dump_line;
            current_line = current_line->next;
            current_line->next = nullptr;
            sprintf(current_line->line, "----- END OF CRASH DUMP -----");
        }
    }

public:

    /**
     * Crash dump formatted as follows:
     * System exited at time: hh:mm:ss
     * Fault type: <fault type>
     * Executing from address: 0x<address>
     */

    CrashParser() = default;

    explicit operator bool() {
        return isvalid(crash_info);
    }

    void generate_crash_dump() {
        if (!isvalid(crash_info)){
            sprintf(current_line->line, "No crash info available");
            return;
        }
        sprintf(current_line->line, "---- CRASH INFO AVAILABLE ----");
        next_line();
        sprintf(current_line->line, "MCIU exited at time: %02lu:%02lu:%02lu",
                (crash_info->time / 3600) % 24, (crash_info->time / 60) % 60, crash_info->time % 60);
        next_line();
        sprintf(current_line->line, "Executing from address: 0x%08lX", crash_info->ret);
        // Add how to find what method is at that address
        next_line();
        sprintf(current_line->line, "To find the method at this address, use the following command:");
        next_line();
        sprintf(current_line->line, "addr2line -e .pio/build/teensy40/firmware.elf 0x%08lX", crash_info->ret);
        next_line();
        if (!csfr_to_string(crash_info)) {
            sprintf(current_line->line, "Fault type: (UNKNWN) Unknown fault type");
        }

        // Clear the crash info
        crash_info->len = 0;
    }

    // Create iterator for crash dump
    char* crash_dump() {
        char* line_ptr = output->line;
        output = output->next;
        return line_ptr;
    }

};

#endif //PRIMROSE_MCIU_CRASHPARSER_H
