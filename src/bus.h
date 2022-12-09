#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cpu.h"

#define RAM_SIZE (1024 * 64)

typedef struct bus {
    uint8_t mem[RAM_SIZE];
    cpu_t *cpu;
} bus_t;

uint8_t bus_read(bus_t *bus, uint16_t addr, bool rom);
void bus_write(bus_t *bus, uint16_t addr, uint8_t data);
