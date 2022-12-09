#include <stdbool.h>
#include <stdint.h>

#include "bus.h"
#include "cpu.h"

void cpu_connect_bus(cpu_t *cpu, bus_t *bus) {
    cpu->ram = bus;
    bus->cpu = cpu;
}

uint8_t cpu_read(cpu_t *cpu, uint16_t addr) {
    return bus_read(cpu->ram, addr, false);
}

void cpu_write(cpu_t *cpu, uint16_t addr, uint8_t data) {
    bus_write(cpu->ram, addr, data);
}

void cpu_clock(cpu_t *cpu) {
    // if no cycles left, we are ready for a new instruction
    // for now we forego doing each clock cycle individually
    // we will treat each instruction as 1 clock cycle (even though it isn't)
    if (cpu->cycles_rem == 0) {
        uint8_t op = cpu_read(cpu, cpu->pc++);          // read new operation and increment pc
        instr_t *instr = &isa[op & 0xf0][op & 0x0f];    // index associated instruction

        // set appropriate fields
        cpu->op = op;
        cpu->cycles_rem = instr->cycles;
        cpu->addr_mode = instr->mode;

        // execute instruction (if valid)
        if (cpu->addr_mode != INVALID)
            instr->func(cpu);
        
    } else {

    }
}

uint8_t fetch(cpu_t *cpu) {
    return 0xff;
}

uint8_t cpu_adc(cpu_t *cpu) {
    return 0;
}

void cpu_status(cpu_t *cpu, uint8_t flag, bool set) {
    if (set)
        cpu->status |= flag;
    else
        cpu->status &= ~flag;
}

uint8_t cpu_and(cpu_t *cpu) {
    uint8_t operand = fetch(cpu);
    cpu->a &= operand;
    cpu_status(cpu, STATUS_ZERO, cpu->a == 0);
    cpu_status(cpu, STATUS_NEGATIVE, cpu->a & (1 << 7));
    return 1;
}


