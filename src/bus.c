#include <stdbool.h>
#include <stdint.h>

#include "bus.h"

uint8_t bus_read(bus_t *bus, uint16_t addr, bool rom) {
    if (addr >= 0 && addr <= 0xffff)
        return bus->mem[addr];
    // if invalid read then return 0 by default
    return 0;
}

void bus_write(bus_t *bus, uint16_t addr, uint8_t data) {
    // for now limit the range to 2kb, but can change this later
    if (addr >= 0 && addr <= 0xffff)
        bus->mem[addr] = data;
}
