/* I have to credit javidx9 for teaching me a lot about NES emulation and for providing helpful
 * source code that I have referenced for help countless times as I embarked upon this endeavor.
 * The datasheet for the 6502 processor is essential reading and will help you make sense of 
 * what this file actually does
 * http://archive.6502.org/datasheets/rockwell_r650x_r651x.pdf
*/
#pragma once

// flagged bits for the status register
#define STATUS_CARRY    (1 << 0)
#define STATUS_ZERO     (1 << 1)
#define STATUS_IRQ      (1 << 2)
#define STATUS_DECIMAL  (1 << 3)
#define STATUS_BRK      (1 << 4)
#define STATUS_OVERFLOW (1 << 6)
#define STATUS_NEGATIVE (1 << 7)

// addressing modes
typedef enum addrmode {
    ACCUMULATOR,
    IMMEDIATE,
    ABSOLUTE,
    ZEROPAGE,
    IDXZEROPAGEX,
    IDXZEROPAGEY,
    IDXABSOLUTEX,
    IDXABSOLUTEY,
    IMPLIED,
    RELATIVE,
    IDXINDIRECTX,
    IDXINDIRECTY,
    ABSOLUTEINDIRECT,
    INVALID
} addrmode_t;

// emulated 6502 cpu
typedef struct cpu {
    uint8_t a;      // accumulator
    uint8_t y;      // y offset
    uint8_t x;      // x offset
    uint16_t pc;    // program counter
    uint8_t sp;     // stack pointer
    uint8_t status; // status register

    bus_t *ram;     // ram

    uint8_t op;             // opcode
    addrmode_t addr_mode;   // addressing mode
    uint8_t cycles_rem;     // number of cycles remaining on current instruction
    uint64_t cycles_count;  // total number of instructions executed by the cpu
} cpu_t;

// 6502 cpu instruction
typedef struct instr {
    const char *name;           // name
    const uint8_t op;           // opcode
    const addrmode_t mode;      // addressing mode
    const uint8_t bytes;        // how many bytes need to be read for this instruction
    const uint8_t cycles;       // clock cycles operation takes
    uint8_t (*func)(cpu_t *);   // associated function
} instr_t;

// POSSIBLY UNUSED BECAUSE OF RESET
void cpu_init(cpu_t *cpu);

// POSSIBLY BETTER WAY TO DO THIS
void cpu_connect_bus(cpu_t *cpu, bus_t *bus);

// interrupts
void cpu_clock(cpu_t *cpu);     // execute one clock cycle
void cpu_irq(cpu_t *cpu);       // send interrupt request
void cpu_nmi(cpu_t *cpu);       // non-maskable irq
void cpu_reset(cpu_t *cpu);     // reset cpu to known state

// isa
uint8_t cpu_adc(cpu_t *cpu);    // add memory to accumulator with carry
uint8_t cpu_and(cpu_t *cpu);    // and memory with accumulator
uint8_t cpu_asl(cpu_t *cpu);    // shift left 1 bit
uint8_t cpu_bcc(cpu_t *cpu);    // branch on carry clear
uint8_t cpu_bcs(cpu_t *cpu);    // branch on carry set
uint8_t cpu_beq(cpu_t *cpu);    // branch on result 0
uint8_t cpu_bit(cpu_t *cpu);    // test memory bits with accumulator
uint8_t cpu_bmi(cpu_t *cpu);    // branch on result minus
uint8_t cpu_bne(cpu_t *cpu);    // branch on result not zero
uint8_t cpu_bpl(cpu_t *cpu);    // branch on result plus
uint8_t cpu_brk(cpu_t *cpu);    // force break
uint8_t cpu_bvc(cpu_t *cpu);    // branch on overflow clear
uint8_t cpu_bvs(cpu_t *cpu);    // branch on overflow set
uint8_t cpu_clc(cpu_t *cpu);    // clear carry flag
uint8_t cpu_cld(cpu_t *cpu);    // clear decimal mode
uint8_t cpu_cli(cpu_t *cpu);    // clear interrupt disable bit
uint8_t cpu_clv(cpu_t *cpu);    // clear overflow flag
uint8_t cpu_cmp(cpu_t *cpu);    // compare memory and accumulator
uint8_t cpu_cpx(cpu_t *cpu);    // compare memory and index x
uint8_t cpu_cpy(cpu_t *cpu);    // compary memory and index y
uint8_t cpu_dec(cpu_t *cpu);    // decrement by one
uint8_t cpu_dex(cpu_t *cpu);    // decrement index x by one
uint8_t cpu_dey(cpu_t *cpu);    // decrement index y by one
uint8_t cpu_eor(cpu_t *cpu);    // xor memory with accumulator
uint8_t cpu_inc(cpu_t *cpu);    // increment by one
uint8_t cpu_inx(cpu_t *cpu);    // increment index x by one
uint8_t cpu_iny(cpu_t *cpu);    // increment index y by one
uint8_t cpu_jmp(cpu_t *cpu);    // jump to new location
uint8_t cpu_jsr(cpu_t *cpu);    // jump to new location saving return addr
uint8_t cpu_lda(cpu_t *cpu);    // load accumulator with memory
uint8_t cpu_ldx(cpu_t *cpu);    // load index x with memory
uint8_t cpu_ldy(cpu_t *cpu);    // load index y with memory
uint8_t cpu_lsr(cpu_t *cpu);    // shift one bit right
uint8_t cpu_nop(cpu_t *cpu);    // no op
uint8_t cpu_ora(cpu_t *cpu);    // or memory with accumulator
uint8_t cpu_pha(cpu_t *cpu);    // push accumulator on stack
uint8_t cpu_php(cpu_t *cpu);    // push processor status on stack
uint8_t cpu_pla(cpu_t *cpu);    // pull accumulator from stack
uint8_t cpu_plp(cpu_t *cpu);    // pull processor status from stack
uint8_t cpu_rol(cpu_t *cpu);    // rotate one bit left
uint8_t cpu_ror(cpu_t *cpu);    // rotate one bit right
uint8_t cpu_rti(cpu_t *cpu);    // return from interrupt
uint8_t cpu_rts(cpu_t *cpu);    // return from subroutine
uint8_t cpu_sbc(cpu_t *cpu);    // subtract from accumulator with borrow
uint8_t cpu_sec(cpu_t *cpu);    // set carry flag
uint8_t cpu_sed(cpu_t *cpu);    // set decimal mode
uint8_t cpu_sei(cpu_t *cpu);    // set interrupt disable bit
uint8_t cpu_sta(cpu_t *cpu);    // store accumulator in memory
uint8_t cpu_stx(cpu_t *cpu);    // store index x in memory
uint8_t cpu_sty(cpu_t *cpu);    // store index y in memory
uint8_t cpu_tax(cpu_t *cpu);    // transfer accumulator to index x
uint8_t cpu_tay(cpu_t *cpu);    // transfer accumulator to index y
uint8_t cpu_tsx(cpu_t *cpu);    // transfer stack pointer to index x
uint8_t cpu_txa(cpu_t *cpu);    // transfer index x to accumulator
uint8_t cpu_txs(cpu_t *cpu);    // transfer index x to stack pointer
uint8_t cpu_tya(cpu_t *cpu);    // transfer index y to accumulator

// list of instructions indexed by (hex) most sig dig for row and least sig dig for col
instr_t isa[16][16] = {
    {{"BRK", 0X00, IMPLIED, 1, 7, &cpu_brk}, {"ORA", 0X01, IDXINDIRECTX, 2, 6, &cpu_ora}, {"INV", 0X02, INVALID, 0, 0, 0}, {"INV", 0X03, INVALID, 0, 0, 0}, {"INV", 0X04, INVALID, 0, 0, 0}, {"ORA", 0x05, ZEROPAGE, 2, 3, &cpu_ora}, {"ASL", 0X06, ZEROPAGE, 2, 5, &cpu_asl}, {"INV", 0X07, INVALID, 0, 0, 0}, {"PHP", 0X08, IMPLIED, 1, 3, &cpu_php}, {"ORA", 0X09, IMMEDIATE, 2, 2, &cpu_ora}, {"ASL", 0X0A, ACCUMULATOR, 1, 2, &cpu_asl}, {"INV", 0X0B, INVALID, 0, 0, 0}, {"INV", 0X0C, INVALID, 0, 0, 0}, {"ORA", 0XD, ABSOLUTE, 3, 4, &cpu_ora}, {"ASL", 0X0E, ABSOLUTE, 3, 6, &cpu_asl}, {"INV", 0X0F, INVALID, 0, 0, 0}},
    {{"BPL", 0X10, RELATIVE, 2, 2, &cpu_bpl}, {"ORA", 0X11, IDXINDIRECTY, 2, 5, &cpu_ora}, {"INV", 0X12, INVALID, 0, 0, 0}, {"INV", 0X13, INVALID, 0, 0, 0}, {"INV", 0X14, INVALID, 0, 0, 0}, {"ORA", 0X15, IDXZEROPAGEX, 2, 4, &cpu_ora}, {"ASL", 0X16, IDXZEROPAGEX, 2, 6, &cpu_asl}, {"INV", 0X17, INVALID, 0, 0, 0}, {"CLC", 0X18, IMPLIED, 1, 2, &cpu_clc}, {"ORA", 0X19, IDXABSOLUTEY, 3, 4, &cpu_ora}, {"INV", 0X1A, INVALID, 0, 0, 0}, {"INV", 0X1B, INVALID, 0, 0, 0}, {"INV", 0X1C, INVALID, 0, 0, 0}, {"ORA", 0x1D, IDXABSOLUTEX, 3, 4, &cpu_ora}, {"ASL", 0X1E, IDXABSOLUTEX, 3, 7, &cpu_asl}, {"INV", 0X1F, INVALID, 0, 0, 0}},
    {{"JSR", 0X20, ABSOLUTE, 3, 6, &cpu_jsr}, {"AND", 0X21, IDXINDIRECTX, 2, 6, &cpu_and}, {"INV", 0X22, INVALID, 0, 0, 0}, {"INV", 0X23, INVALID, 0, 0, 0}, {"BIT", 0X24, ZEROPAGE, 2, 3, &cpu_bit}, {"AND", 0X25, ZEROPAGE, 2, 3, &cpu_bit}, {"ROL", 0X26, ZEROPAGE, 2, 5, &cpu_rol}, {"INV", 0X27, INVALID, 0, 0, 0}, {"PLP", 0X28, IMPLIED, 1, 4, &cpu_plp}, {"AND", 0X29, IMMEDIATE, 2, 2, &cpu_and}, {"ROL", 0X2A, ACCUMULATOR, 1, 2, &cpu_rol}, {"INV", 0X2B, INVALID, 0, 0, 0}, {"BIT", 0X2C, ABSOLUTE, 3, 4, &cpu_bit}, {"AND", 0X2D, ABSOLUTE, 3, 4, &cpu_and}, {"ROL", 0X2E, ABSOLUTE, 3, 6, &cpu_rol}, {"INV", 0X2F, INVALID, 0, 0, 0}},
    {{"BMI", 0X30, RELATIVE, 2, 2, &cpu_bmi}, {"AND", 0X31, IDXINDIRECTY, 2, 5, &cpu_and}, {"INV", 0X32, INVALID, 0, 0, 0}, {"INV", 0X33, INVALID, 0, 0, 0}, {"INV", 0X34, INVALID, 0, 0, 0}, {"AND", 0X35, IDXZEROPAGEX, 2, 4, &cpu_and}, {"ROL", 0X36, IDXZEROPAGEX, 2, 6, &cpu_rol}, {"INV", 0X37, INVALID, 0, 0, 0}, {"SEC", 0X38, IMPLIED, 1, 2, &cpu_sec}, {"AND", 0X39, IDXABSOLUTEY, 3, 4, &cpu_and}, {"INV", 0X3A, INVALID, 0, 0, 0}, {"INV", 0X3B, INVALID, 0, 0, 0}, {"INV", 0X3C, INVALID, 0, 0, 0}, {"AND", 0X3D, IDXABSOLUTEX, 3, 4, &cpu_and}, {"ROL", 0X3E, IDXABSOLUTEX, 3, 7, &cpu_rol}, {"INV", 0X3F, INVALID, 0, 0, 0}},
    {{"RTI", 0X40, IMPLIED, 1, 6, &cpu_rti}, {"EOR", 0X41, IDXINDIRECTX, 2, 6, &cpu_eor}, {"INV", 0X42, INVALID, 0, 0, 0}, {"INV", 0X43, INVALID, 0, 0, 0}, {"INV", 0X44, INVALID, 0, 0, 0}, {"EOR", 0X45, ZEROPAGE, 2, 3, &cpu_eor}, {"LSR", 0X46, ZEROPAGE, 2, 5, &cpu_lsr}, {"INV", 0X47, INVALID, 0, 0, 0}, {"PHA", 0X48, IMPLIED, 1, 3, &cpu_pha}, {"EOR", 0X49, IMMEDIATE, 2, 2, &cpu_eor}, {"LSR", 0X4A, ACCUMULATOR, 1, 2, &cpu_lsr}, {"INV", 0X4B, INVALID, 0, 0, 0}, {"JMP", 0X4C, ABSOLUTE, 3, 3, &cpu_jmp}, {"EOR", 0X4D, ABSOLUTE, 3, 4, &cpu_eor}, {"LSR", 0X4E, ABSOLUTE, 3, 6, &cpu_lsr}, {"INV", 0X4F, INVALID, 0, 0, 0}},
    {{"BVC", 0X50, RELATIVE, 2, 2, &cpu_bvc}, {"EOR", 0X51, IDXINDIRECTY, 2, 5, &cpu_eor}, {"INV", 0X52, INVALID, 0, 0, 0}, {"INV", 0X53, INVALID, 0, 0, 0}, {"INV", 0X54, INVALID, 0, 0, 0}, {"EOR", 0X55, IDXZEROPAGEX, 2, 4, &cpu_eor}, {"LSR", 0X56, IDXZEROPAGEX, 2, 6, &cpu_lsr}, {"INV", 0X57, INVALID, 0, 0, 0}, {"CLI", 0X58, IMPLIED, 1, 2, &cpu_cli}, {"EOR", 0X59, IDXABSOLUTEY, 3, 4, &cpu_eor}, {"INV", 0X5A, INVALID, 0, 0, 0}, {"INV", 0X5B, INVALID, 0, 0, 0}, {"INV", 0X5C, INVALID, 0, 0, 0}, {"EOR", 0X5D, IDXABSOLUTEX, 3, 4, &cpu_eor}, {"LSR", 0X5E, IDXABSOLUTEX, 3, 7, &cpu_lsr}, {"INV", 0X5F, INVALID, 0, 0, 0}},
    {{"RTS", 0X60, IMPLIED, 1, 6, &cpu_rts}, {"ADC", 0X61, IDXINDIRECTX, 2, 6, &cpu_adc}, {"INV", 0X62, INVALID, 0, 0, 0}, {"INV", 0X63, INVALID, 0, 0, 0}, {"INV", 0X64, INVALID, 0, 0, 0}, {"ADC", 0X65, ZEROPAGE, 2, 3, &cpu_adc}, {"ROR", 0X66, ZEROPAGE, 2, 5, &cpu_ror}, {"INV", 0X67, INVALID, 0, 0, 0}, {"PLA", 0X68, IMPLIED, 1, 4, &cpu_pla}, {"ADC", 0X69, IMMEDIATE, 2, 2, &cpu_adc}, {"ROR", 0X6A, ACCUMULATOR, 1, 2, &cpu_ror}, {"INV", 0X6B, INVALID, 0, 0, 0}, {"JMP", 0X6C, ABSOLUTEINDIRECT, 3, 5, &cpu_jmp}, {"ADC", 0X6D, ABSOLUTE, 3, 4, &cpu_adc}, {"ROR", 0X6E, ABSOLUTE, 3, 6, &cpu_ror}, {"INV", 0X6F, INVALID, 0, 0, 0}},
    {{"BVS", 0X70, RELATIVE, 2, 2, &cpu_bvs}, {"ADC", 0X71, IDXINDIRECTY, 2, 5, &cpu_adc}, {"INV", 0X72, INVALID, 0, 0, 0}, {"INV", 0X73, INVALID, 0, 0, 0}, {"INV", 0X74, INVALID, 0, 0, 0}, {"ADC", 0X75, IDXZEROPAGEX, 2, 4, &cpu_adc}, {"ROR", 0X76, IDXZEROPAGEX, 2, 6, &cpu_ror}, {"INV", 0X77, INVALID, 0, 0, 0}, {"SEI", 0X78, IMPLIED, 1, 2, &cpu_sei}, {"ADC", 0X79, IDXABSOLUTEY, 3, 4, &cpu_adc}, {"INV", 0X7A, INVALID, 0, 0, 0}, {"INV", 0X7B, INVALID, 0, 0, 0}, {"INV", 0X7C, INVALID, 0, 0, 0}, {"ADC", 0X7D, IDXABSOLUTEX, 3, 4, &cpu_adc}, {"ROR", 0X7E, IDXABSOLUTEX, 3, 7, &cpu_ror}, {"INV", 0X7F, INVALID, 0, 0, 0}},
    {{"INV", 0X80, INVALID, 0, 0, 0}, {"STA", 0X81, IDXINDIRECTX, 2, 6, &cpu_sta}, {"INV", 0X82, INVALID, 0, 0, 0}, {"INV", 0X83, INVALID, 0, 0, 0}, {"STY", 0X84, ZEROPAGE, 2, 3, &cpu_sty}, {"STA", 0X85, ZEROPAGE, 2, 3, &cpu_sta}, {"STX", 0X86, ZEROPAGE, 2, 3, &cpu_stx}, {"INV", 0X87, INVALID, 0, 0, 0}, {"DEY", 0X88, IMPLIED, 1, 2, &cpu_dey}, {"INV", 0X89, INVALID, 0, 0, 0}, {"TXA", 0X8A, IMPLIED, 1, 2, &cpu_txa}, {"INV", 0X8B, INVALID, 0, 0, 0}, {"STY", 0X8C, ABSOLUTE, 3, 4, &cpu_sty}, {"STA", 0X8D, ABSOLUTE, 3, 4, &cpu_sta}, {"STX", 0X8E, ABSOLUTE, 3, 4, &cpu_stx}, {"INV", 0X8F, INVALID, 0, 0, 0}},
    {{"BCC", 0X90, RELATIVE, 2, 2, &cpu_bcc}, {"STA", 0X91, IDXINDIRECTY, 2, 6, &cpu_sta}, {"INV", 0X92, INVALID, 0, 0, 0}, {"INV", 0X93, INVALID, 0, 0, 0}, {"STY", 0X94, IDXZEROPAGEX, 2, 4, &cpu_sty}, {"STA", 0X95, IDXZEROPAGEX, 2, 4, &cpu_sta}, {"STX", 0X96, IDXZEROPAGEY, 2, 4, &cpu_stx}, {"INV", 0X97, INVALID, 0, 0, 0}, {"TYA", 0X98, IMPLIED, 1, 2, &cpu_tya}, {"STA", 0X99, IDXABSOLUTEY, 3, 5, &cpu_sta}, {"TXS", 0X9A, IMPLIED, 1, 2, &cpu_txs}, {"INV", 0X9B, INVALID, 0, 0, 0}, {"INV", 0X9C, INVALID, 0, 0, 0}, {"STA", 0X9D, IDXABSOLUTEX, 3, 5, &cpu_sta}, {"INV", 0X9E, INVALID, 0, 0, 0}, {"INV", 0X9F, INVALID, 0, 0, 0}},
    {{"LDY", 0XA0, IMMEDIATE, 2, 2, &cpu_ldy}, {"LDA", 0XA1, IDXINDIRECTX, 2, 6, &cpu_lda}, {"LDX", 0XA2, IMMEDIATE, 2, 2, &cpu_ldx}, {"INV", 0XA3, INVALID, 0, 0, 0}, {"LDY", 0XA4, ZEROPAGE, 2, 3, &cpu_ldy}, {"LDA", 0XA5, ZEROPAGE, 2, 3, &cpu_lda}, {"LDX", 0XA6, ZEROPAGE, 2, 4, &cpu_ldx}, {"INV", 0XA7, INVALID, 0, 0, 0}, {"TAY", 0XA8, IMPLIED, 1, 2, &cpu_tay}, {"LDA", 0XA9, IMMEDIATE, 2, 2, &cpu_lda}, {"TAX", 0XAA, IMPLIED, 1, 2, &cpu_tax}, {"INV", 0XAB, INVALID, 0, 0, 0}, {"LDY", 0XAC, ABSOLUTE, 3, 4, &cpu_ldy}, {"LDA", 0XAD, ABSOLUTE, 3, 4, &cpu_lda}, {"LDX", 0XAE, ABSOLUTE, 3, 4, &cpu_ldx}, {"INV", 0XAF, INVALID, 0, 0, 0}},
    {{"BCS", 0XB0, RELATIVE, 2, 2, &cpu_bcs}, {"LDA", 0XB1, IDXINDIRECTY, 2, 5, &cpu_lda}, {"INV", 0XB2, INVALID, 0, 0, 0}, {"INV", 0XB3, INVALID, 0, 0, 0}, {"LDY", 0XB4, IDXZEROPAGEX, 2, 4, &cpu_ldy}, {"LDA", 0XB5, IDXZEROPAGEX, 2, 4, &cpu_lda}, {"LDX", 0XB6, IDXZEROPAGEY, 2, 4, &cpu_ldx}, {"INV", 0XB7, INVALID, 0, 0, 0}, {"CLV", 0XB8, IMPLIED, 1, 2, &cpu_clv}, {"LDA", 0XB9, IDXABSOLUTEY, 3, 4, &cpu_lda}, {"TSX", 0XBA, IMPLIED, 1, 2, &cpu_tsx}, {"INV", 0XBB, INVALID, 0, 0, 0}, {"LDY", 0XBC, IDXABSOLUTEX, 3, 4, &cpu_ldy}, {"LDA", 0XBD, IDXABSOLUTEX, 3, 4, &cpu_lda}, {"LDX", 0XBE, IDXABSOLUTEY, 3, 4, &cpu_ldx}, {"INV", 0XBF, INVALID, 0, 0, 0}},
    {{"CPY", 0XC0, IMMEDIATE, 2, 2, &cpu_cpy}, {"CMP", 0XC1, IDXINDIRECTX, 2, 6, &cpu_cmp}, {"INV", 0XC2, INVALID, 0, 0, 0}, {"INV", 0XC3, INVALID, 0, 0, 0}, {"CPY", 0XC4, ZEROPAGE, 2, 3, &cpu_cpy}, {"CMP", 0XC5, ZEROPAGE, 2, 3, &cpu_cmp}, {"DEC", 0XC6, ZEROPAGE, 2, 5, &cpu_dec}, {"INV", 0XC7, INVALID, 0, 0, 0}, {"INY", 0XC8, IMPLIED, 1, 2, &cpu_iny}, {"CMP", 0XC9, IMMEDIATE, 2, 2, &cpu_cmp}, {"DEX", 0XCA, IMPLIED, 1, 2, &cpu_dex}, {"INV", 0XCB, INVALID, 0, 0, 0}, {"CPY", 0XCC, ABSOLUTE, 3, 4, &cpu_cpy}, {"CMP", 0XCD, ABSOLUTE, 3, 4, &cpu_cmp}, {"DEC", 0XCE, ABSOLUTE, 3, 6, &cpu_dec}, {"INV", 0XCF, INVALID, 0, 0, 0}},
    {{"BNE", 0XD0, RELATIVE, 2, 2, &cpu_bne}, {"CMP", 0XD1, IDXINDIRECTY, 2, 5, &cpu_cmp}, {"INV", 0XD2, INVALID, 0, 0, 0}, {"INV", 0XD3, INVALID, 0, 0, 0}, {"INV", 0XD4, INVALID, 0, 0, 0}, {"CMP", 0XD5, IDXZEROPAGEX, 2, 4, &cpu_cmp}, {"DEC", 0XD6, IDXZEROPAGEX, 2, 6, &cpu_dec}, {"INV", 0XD7, INVALID, 0, 0, 0}, {"CLD", 0XD8, IMPLIED, 1, 2, &cpu_cld}, {"CMP", 0XD9, IDXABSOLUTEY, 3, 4, &cpu_cmp}, {"INV", 0XDA, INVALID, 0, 0, 0}, {"INV", 0XDB, INVALID, 0, 0, 0}, {"INV", 0XDC, INVALID, 0, 0, 0}, {"CMP", 0XDD, IDXABSOLUTEX, 3, 4, &cpu_cmp}, {"DEC", 0XDE, IDXABSOLUTEX, 3, 7, &cpu_dec}, {"INV", 0XDF, INVALID, 0, 0, 0}},
    {{"CPX", 0XE0, IMMEDIATE, 2, 2, &cpu_cpx}, {"SBC", 0XE1, IDXINDIRECTX, 2, 6, &cpu_sbc}, {"INV", 0XE2, INVALID, 0, 0, 0}, {"INV", 0XE3, INVALID, 0, 0, 0}, {"CPX", 0XE4, ZEROPAGE, 2, 3, &cpu_cpx}, {"SBC", 0XE5, ZEROPAGE, 2, 3, &cpu_sbc}, {"INC", 0XE6, ZEROPAGE, 2, 5, &cpu_inc}, {"INV", 0XE7, INVALID, 0, 0, 0}, {"INX", 0XE8, IMPLIED, 1, 2, &cpu_inx}, {"SBC", 0XE9, IMMEDIATE, 2, 2, &cpu_sbc}, {"NOP", 0XEA, IMPLIED, 1, 2, &cpu_nop}, {"INV", 0XEB, INVALID, 0, 0, 0}, {"CPX", 0XEC, ABSOLUTE, 3, 4, &cpu_cpx}, {"SBC", 0XED, ABSOLUTE, 3, 4, &cpu_sbc}, {"INC", 0XEE, ABSOLUTE, 3, 6, &cpu_inc}, {"INV", 0XEF, INVALID, 0, 0, 0}},
    {{"BEQ", 0XF0, RELATIVE, 2, 2, &cpu_beq}, {"SBC", 0XF1, IDXINDIRECTY, 2, 5, &cpu_sbc}, {"INV", 0XF2, INVALID, 0, 0, 0}, {"INV", 0XF3, INVALID, 0, 0, 0}, {"INV", 0XF4, INVALID, 0, 0, 0}, {"SBC", 0XF5, IDXZEROPAGEX, 2, 4, &cpu_sbc}, {"INC", 0XF6, IDXZEROPAGEX, 2, 6, &cpu_inc}, {"INV", 0XF7, INVALID, 0, 0, 0}, {"SED", 0XF8, IMPLIED, 1, 2, &cpu_sed}, {"SBC", 0XF9, IDXABSOLUTEY, 3, 4, &cpu_sbc}, {"INV", 0XFA, INVALID, 0, 0, 0}, {"INV", 0XFB, INVALID, 0, 0, 0}, {"INV", 0XFC, INVALID, 0, 0, 0}, {"SBC", 0XFD, IDXABSOLUTEX, 3, 4, &cpu_sbc}, {"INC", 0XFE, IDXABSOLUTEX, 3, 7, &cpu_inc}, {"INV", 0XFF, INVALID, 0, 0, 0}}
};

// helpers
// POSSIBLY UNUSED
bool cpu_instr_complete(cpu_t *cpu);

// TODO: add dissassembler logic for debugging

uint8_t cpu_fetch(cpu_t *cpu);

void cpu_status(cpu_t *cpu, uint8_t flag, bool set);

uint8_t cpu_read(cpu_t *cpu, uint16_t addr);
void cpu_write(cpu_t *cpu, uint16_t addr, uint8_t data);

