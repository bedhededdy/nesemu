#include <stdio.h>

#include "cpu.h"

int main() {
    cpu_t cpu;
    cpu.a = 0x3;
    cpu_and(&cpu);


    printf("%x\n", cpu.a);
    return 0;
}
