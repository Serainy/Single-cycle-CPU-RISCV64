`ifndef DIFINES_SV
`define DIFINES_SV
`ifdef VERILATOR
`include "include/config.sv"
`endif

import config_pkg::*;
package defines;
	import config_pkg::*;
	`define     REG_WIDTH           64
    `define     INSTR_WIDTH         32
    `define     REG_COUNT           32
    `define     REG_ADDR_WIDTH      5
    `define     ALUCTRL_WIDTH       3
    `define     ALUOP_WIDTH         2

    // 指令的14到12位funct3
    `define     ADD                 3'b000      // ADD,SUB, MUL
    `define     XOR                 3'b100      // XOR,     DIV
    `define     OR                  3'b110      // OR,      REM,    CSRRSI
    `define     AND                 3'b111      // AND,     REMU,   CSRRCI
    `define     SLL                 3'b001      // SLL,             CSRRW
    `define     SRL_SRA             3'b101      // SRA,SRL, DIVU,   CSRRWI
    `define     SLT                 3'b010      // SLT,             CSRRS
    `define     SLTU                3'b011      // SLTU,            CSRRC
    

    // lab1
    `define     INSTR_R        7'b0110011
    `define     INSTR_I        7'b0010011
    `define     INSTR_B        7'b1100011
    `define     INSTR_J        7'b1101111
    `define     INSTR_JR       7'b1100111
    `define     INSTR_U        7'b0110111
    `define     INSTR_UPC      7'b0010111
    // lab2
    `define     INSTR_S        7'b0100011
    `define     INSTR_IL       7'b0000011
    // lab4
    `define     INSTR_RW       7'b0111011
    `define     INSTR_IW       7'b0011011
    // lab_last
    `define     INSTR_CSR      7'b1110011

    typedef struct packed {
        logic [3:0]  mode;  // [63:60]
        logic [15:0] asid;  // [59:44]
        logic [43:0] ppn;   // [43:0]
    } satp_t;

    typedef struct packed {
        logic [63:0]  mstatus;    
        logic [63:0]  mtvec;    
        logic [63:0]  mip;    
        logic [63:0]  mie;    
        logic [63:0]  mscratch;    
        logic [63:0]  mcause;    
        logic [63:0]  mtval;    
        logic [63:0]  mepc;    
        logic [63:0]  mcycle;    
        satp_t satp;
        logic [1:0]   mode;
    } CSR_reg;

endpackage
`endif
