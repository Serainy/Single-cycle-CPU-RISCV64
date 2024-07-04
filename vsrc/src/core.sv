`ifndef __CORE_SV
`define __CORE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/defines.sv"
`endif

module adder
    import defines::*;(
    input  logic [`REG_WIDTH-1:0] a,
    input  logic [`REG_WIDTH-1:0] b,
    output logic [`REG_WIDTH-1:0] y
    );
    assign y = a + b;
	endmodule

module instr_reg
	import defines::*;(
	input  logic 					clk,
	input  logic 					rst,
	input  logic 					hand_in,
	input  logic 					iresp_dataok,
	input  logic [`INSTR_WIDTH-1:0] iresp_data,
	output logic [`INSTR_WIDTH-1:0] instr,
	output logic [`INSTR_WIDTH-1:0] instr_reg
	);
    always_ff @(posedge clk, posedge rst)
    if (rst)   
		instr_reg <= `INSTR_WIDTH'b0;
    else if(hand_in)
	    instr_reg <= `INSTR_WIDTH'b0;
    else if(iresp_dataok)
	    instr_reg <= iresp_data;
	else
		instr_reg <= instr_reg;
	assign instr = (iresp_dataok == 0) ? instr_reg : iresp_data;

	endmodule

module csr
	import defines::*;(
    input  logic clk,
    input  logic rst,
    input  logic csr_we,
    input  logic [11:0] 			csr_addr, 
    input  logic [`REG_WIDTH-1:0]  	csr_wdata, 
    output logic [`REG_WIDTH-1:0]  	csr_rdata,

    output CSR_reg 					csr_reg,

    input  logic  					is_mret,

    input  logic [`REG_WIDTH-1:0]  	pc,
	output logic [`REG_WIDTH-1:0]  	csr_pc,
	input  logic 					instr_unaligned, 
	input  logic 					data_unaligned_r, 
	input  logic 					data_unaligned_w, 
	input  logic 					illegal_instr, 
	input  logic 					ecall, 

	input  logic 					trint,
	input  logic 					swint, 
	input  logic 					exint,
	output logic 					is_csr,

	input  logic					hand_in
	);
    // localparam logic [11:0]
    //     	ADDR_MSTATUS  = 12'h300,
    //     	ADDR_MTVEC    = 12'h305,
    //     	ADDR_MIP      = 12'h344,
    //     	ADDR_MIE      = 12'h304,
    //     	ADDR_MSCRATCH = 12'h340,
    //     	ADDR_MCAUSE   = 12'h342,
    //     	ADDR_MTVAL    = 12'h343,
    //     	ADDR_MEPC     = 12'h341,
    //     	ADDR_MCYCLE   = 12'hB00,
    //     	ADDR_SATP     = 12'h180,
	// 		ADDR_MODE 	  = 12'h7C0;
    localparam logic [11:0]
        ADDR_MSTATUS_A  = 12'h300,
        ADDR_MTVEC_A    = 12'h305,
        ADDR_MIP_A      = 12'h344,
        ADDR_MIE_A      = 12'h304,
        ADDR_MSCRATCH_A = 12'h340,
        ADDR_MCAUSE_A   = 12'h342,
        ADDR_MTVAL_A    = 12'h343,
        ADDR_MEPC_A     = 12'h341,
        ADDR_MCYCLE_A   = 12'hB00,
        ADDR_SATP_A     = 12'h180,
		ADDR_MODE_A 	= 12'h7C0;
    localparam logic [3:0]
        ADDR_MSTATUS  	= 'h1,
        ADDR_MTVEC    	= 'h2,
        ADDR_MIP      	= 'h3,
        ADDR_MIE      	= 'h4,
        ADDR_MSCRATCH 	= 'h5,
        ADDR_MCAUSE   	= 'h6,
        ADDR_MTVAL    	= 'h7,
        ADDR_MEPC     	= 'h8,
        ADDR_MCYCLE   	= 'h9,
        ADDR_SATP     	= 'hA,
		ADDR_MODE 	  	= 'hB;
	
	logic [3:0] used;
	always_comb
		case(csr_addr)
			ADDR_MSTATUS_A: used = ADDR_MSTATUS;
			ADDR_MTVEC_A: 	used = ADDR_MTVEC;
			ADDR_MIP_A: 	used = ADDR_MIP;
			ADDR_MIE_A: 	used = ADDR_MIE;
			ADDR_MSCRATCH_A:used = ADDR_MSCRATCH;
			ADDR_MCAUSE_A: 	used = ADDR_MCAUSE;
			ADDR_MTVAL_A: 	used = ADDR_MTVAL;
			ADDR_MEPC_A: 	used = ADDR_MEPC;
			ADDR_MCYCLE_A: 	used = ADDR_MCYCLE;
			ADDR_SATP_A: 	used = ADDR_SATP;
			ADDR_MODE_A: 	used = ADDR_MODE;
			default: 		used = 'h0;
		endcase

    logic [`REG_WIDTH-1:0] csr_regs [11:0];
	logic [`REG_WIDTH-1:0] csr_regs_nxt [11:0]; 

	logic exception_pending, interrupt_pending;
	assign is_csr = exception_pending | interrupt_pending;
    logic [63:0] cause_code;
    
	always @(posedge clk)
	begin
		if(rst)
		begin
			exception_pending <= 0;
			cause_code <= 0;
		end
		else if(hand_in) begin
			exception_pending <= 0;
            cause_code <= 64'h0000000000000000;
		end
        // Check for exceptions
		else if (instr_unaligned) begin
            exception_pending <= 1;
            cause_code <= 64'h0000000000000000; // Instruction address misaligned
        end else if (data_unaligned_r) begin
            exception_pending <= 1;
            cause_code <= 64'h0000000000000004; // Load address misaligned read
        end else if (data_unaligned_w) begin
            exception_pending <= 1;
            cause_code <= 64'h0000000000000006; // Load address misaligned write
        end else if (illegal_instr) begin
            exception_pending <= 1;
            cause_code <= 64'h0000000000000002; // Illegal instruction
        end else if (ecall) begin
            exception_pending <= 1;
            cause_code <= 64'h0000000000000008; // Environment call from M-mode
        end
		if(rst)
		begin
			interrupt_pending <= 0;
		end
        // Check for interrupts
        else if ((trint && csr_reg.mstatus[3] && csr_reg.mie[7]) 
		 	  || (swint && csr_reg.mstatus[3] && csr_reg.mie[3]) 
		 	  || (exint && csr_reg.mstatus[3] && csr_reg.mie[11])) 
		begin
            interrupt_pending <= 1;
            cause_code <= trint ? 64'h8000000000000007 : (swint ? 64'h8000000000000003 : 64'h800000000000000b);
        end
    end

    always_comb begin
		csr_pc = pc + 4;
		for (int i = 0; i < 12; i = i + 1) begin
			csr_regs_nxt[i] = csr_regs[i];
		end
		// csr_regs_nxt[ADDR_MSTATUS] 	= csr_regs[ADDR_MSTATUS];
		// csr_regs_nxt[ADDR_MTVEC] 	= csr_regs[ADDR_MTVEC];
		// csr_regs_nxt[ADDR_MIP] 		= csr_regs[ADDR_MIP];
		// csr_regs_nxt[ADDR_MIE] 		= csr_regs[ADDR_MIE];
		// csr_regs_nxt[ADDR_MSCRATCH] = csr_regs[ADDR_MSCRATCH];
		// csr_regs_nxt[ADDR_MCAUSE] 	= csr_regs[ADDR_MCAUSE]; 
		// csr_regs_nxt[ADDR_MTVAL] 	= csr_regs[ADDR_MTVAL];
		// csr_regs_nxt[ADDR_MEPC] 	= csr_regs[ADDR_MEPC];
		// csr_regs_nxt[ADDR_MCYCLE] 	= csr_regs[ADDR_MCYCLE];
		// csr_regs_nxt[ADDR_SATP] 	= csr_regs[ADDR_SATP];
		// csr_regs_nxt[ADDR_MODE] 	= csr_regs[ADDR_MODE];
		if(exception_pending || interrupt_pending)
		begin
			csr_regs_nxt[ADDR_MEPC] 			= pc;
			csr_pc 								= csr_regs_nxt[ADDR_MTVEC];
			csr_regs_nxt[ADDR_MCAUSE] 			= cause_code;
			csr_regs_nxt[ADDR_MSTATUS][7] 		= csr_regs_nxt[ADDR_MSTATUS][3];
			csr_regs_nxt[ADDR_MSTATUS][3] 		= 0;
			csr_regs_nxt[ADDR_MSTATUS][12:11] 	= csr_regs_nxt[ADDR_MODE][1:0];
			if (ecall)
				csr_regs_nxt[ADDR_MODE] 		= 3;
		end
        else if (csr_we) 
		begin
			if(is_mret)
			begin
				csr_regs_nxt[ADDR_MODE][1:0]  = csr_regs_nxt[ADDR_MSTATUS][12:11];
				csr_regs_nxt[ADDR_MSTATUS][3] = csr_regs_nxt[ADDR_MSTATUS][7];
				csr_regs_nxt[ADDR_MSTATUS][7] = 1;
				csr_regs_nxt[ADDR_MSTATUS][12:11] = 2'b00;
			end
			else
			begin
				csr_regs_nxt[used] = csr_wdata;
			end
        end
    end

    always @(posedge clk or posedge rst) begin
        if(rst) 
		begin
			for (int i = 0; i < 12; i = i + 1) begin
				csr_regs[i] <= '0;
			end
			// csr_regs <= '0;
			// csr_regs[ADDR_MSTATUS]  <= '0;
			// csr_regs[ADDR_MTVEC]    <= '0;
			// csr_regs[ADDR_MIP]      <= '0;
			// csr_regs[ADDR_MIE]      <= '0;
			// csr_regs[ADDR_MSCRATCH] <= '0;
			// csr_regs[ADDR_MCAUSE]   <= '0;
			// csr_regs[ADDR_MTVAL]    <= '0;
			// csr_regs[ADDR_MEPC]     <= '0;
			// csr_regs[ADDR_MCYCLE]   <= '0;
			// csr_regs[ADDR_SATP]     <= '0;
			csr_regs[ADDR_MODE] <= 3;
		end
        else 
		begin
            csr_regs[ADDR_MSTATUS]  <= csr_regs_nxt[ADDR_MSTATUS];
			csr_regs[ADDR_MTVEC]    <= csr_regs_nxt[ADDR_MTVEC];
			csr_regs[ADDR_MIP]      <= csr_regs_nxt[ADDR_MIP];
			csr_regs[ADDR_MIE]      <= csr_regs_nxt[ADDR_MIE];
			csr_regs[ADDR_MSCRATCH] <= csr_regs_nxt[ADDR_MSCRATCH];
			csr_regs[ADDR_MCAUSE]   <= csr_regs_nxt[ADDR_MCAUSE];
			csr_regs[ADDR_MTVAL]    <= csr_regs_nxt[ADDR_MTVAL];
			csr_regs[ADDR_MEPC]     <= csr_regs_nxt[ADDR_MEPC];
			csr_regs[ADDR_MCYCLE]   <= csr_regs_nxt[ADDR_MCYCLE];
			csr_regs[ADDR_SATP]     <= csr_regs_nxt[ADDR_SATP];
			csr_regs[ADDR_MODE]     <= csr_regs_nxt[ADDR_MODE];
        end
    end

    assign csr_rdata 		= csr_regs[used];


    assign csr_reg.mstatus  = csr_regs_nxt[ADDR_MSTATUS];
    assign csr_reg.mtvec    = csr_regs_nxt[ADDR_MTVEC];
    assign csr_reg.mip      = csr_regs_nxt[ADDR_MIP];
    assign csr_reg.mie      = csr_regs_nxt[ADDR_MIE];
    assign csr_reg.mscratch = csr_regs_nxt[ADDR_MSCRATCH];
    assign csr_reg.mcause   = csr_regs_nxt[ADDR_MCAUSE];
    assign csr_reg.mtval    = csr_regs_nxt[ADDR_MTVAL];
    assign csr_reg.mepc     = csr_regs_nxt[ADDR_MEPC];
    assign csr_reg.mcycle   = csr_regs_nxt[ADDR_MCYCLE];
    assign csr_reg.satp     = csr_regs_nxt[ADDR_SATP];
    assign csr_reg.mode     = csr_regs_nxt[ADDR_MODE][1:0];

	endmodule

module multiplier_multicycle 
	import defines::*;(
    input  logic clk, rst, valid,
    input  logic [`REG_WIDTH-1:0] a, b,
    output logic [`REG_WIDTH-1:0] c,
    output logic done
	);
	typedef enum logic { INIT, DOING } state_t;
	state_t state, state_nxt;
    assign done = (state_nxt == INIT) & valid;

    logic [`REG_WIDTH+1:0] count, count_nxt;
    localparam logic [`REG_WIDTH+1:0] MULT_DELAY = {1'b0, 1'b1, 64'b0};

    always_ff @(posedge clk) begin
        if (rst) begin
            {state, count} <= '0;
        end else begin
            {state, count} <= {state_nxt, count_nxt};
        end
    end

    always_comb begin
        {state_nxt, count_nxt} = {state, count}; // default
        unique case(state)
            INIT: begin
                if (valid) begin
                    state_nxt = DOING;
                    count_nxt = MULT_DELAY;
                end
            end
            DOING: begin
                count_nxt = {1'b0, count_nxt[65:1]};
                if (count_nxt == '0) begin
                    state_nxt = INIT;
                end
            end
			default:
				if (valid) begin
                    state_nxt = DOING;
                    count_nxt = MULT_DELAY;
                end
        endcase
    end
    logic [128:0] p, p_nxt;
    always_comb begin
        p_nxt = p;
        unique case(state)
            INIT: begin
                p_nxt = {65'b0, a};
            end
            DOING: begin
                if (p_nxt[0]) begin
                    p_nxt[128:64] = p_nxt[127:64] + b;
                    // p_nxt[64:32] = p_nxt[64:32] + b;
            	end
            	p_nxt = {1'b0, p_nxt[128:1]};
            end
			default:
				p_nxt = {65'b0, a};
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            p <= '0;
        end else begin
            p <= p_nxt;
        end
    end
    assign c = p[63:0];
	endmodule

module divider_multicycle
	import defines::*; #(parameter WIDTH = 64)(
    input  logic clk, rst, valid_div, valid_divu, valid_rem, valid_remu, valid_w,
    input  logic [WIDTH-1:0] a, b,
    output logic [WIDTH-1:0] c, // c = {a % b, a / b}
    output logic done
	);
	logic flag;
	assign flag = (WIDTH == 64)? ~valid_w:valid_w;
    typedef enum logic { INIT, DOING } state_t;
	state_t state, state_nxt;
    logic [WIDTH+1:0] count, count_nxt;
    // localparam logic [WIDTH+1:0] 	DIV_DELAY = {1'b0, 1'b1, WIDTH{1'b0}};
    logic [WIDTH+1:0] 	DIV_DELAY;
	assign 						 	DIV_DELAY[WIDTH-1:0] 		= 0;
	assign							DIV_DELAY[WIDTH+1:WIDTH] 	= 1;

	logic  isUnsigned;
	assign isUnsigned = valid_divu | valid_remu;
	logic  [WIDTH-1:0] a_abs, b_abs;
	assign a_abs = (isUnsigned == 0)?((a[WIDTH-1] == 0)?a:(~a+1)):a;
	assign b_abs = (isUnsigned == 0)?((b[WIDTH-1] == 0)?b:(~b+1)):b;
	logic  isDiv_minus, isRem_minus;
	assign isDiv_minus = a[WIDTH-1] != b[WIDTH-1];
	assign isRem_minus = (a[WIDTH-1] == 1); // a < 0

    always_ff @(posedge clk) begin
        if (rst) begin
            {state, count} <= '0;
        end else begin
            {state, count} <= {state_nxt, count_nxt};
        end
    end
    assign done = (state_nxt == INIT);
    always_comb begin
        {state_nxt, count_nxt} = {state, count}; // default
        unique case(state)
            INIT: begin
                if ((valid_div | valid_divu | valid_rem | valid_remu) & flag) begin
                    state_nxt = DOING;
                    count_nxt = DIV_DELAY;
                end
            end
            DOING: begin
                count_nxt = {1'b0, count_nxt[WIDTH+1:1]};
                if (count_nxt == '0) begin
                    state_nxt = INIT;
                end
            end
			default: begin
                if ((valid_div | valid_divu | valid_rem | valid_remu) & flag) begin
                    state_nxt = DOING;
                    count_nxt = DIV_DELAY;
                end
            end
        endcase
    end
    logic [2*WIDTH-1:0] p, p_nxt;
    always_comb begin
        p_nxt = p;
        unique case(state)
            INIT: begin
				p_nxt[WIDTH-1:0] 		= a_abs;
				p_nxt[2*WIDTH-1:WIDTH]	= 0;
                // p_nxt = {WIDTH'b0, a_abs};
            end
            DOING: begin
                p_nxt = {p_nxt[2*WIDTH-2:0], 1'b0};
                if (p_nxt[2*WIDTH-1:WIDTH] >= b_abs) begin
                    p_nxt[2*WIDTH-1:WIDTH] -= b_abs;
                    p_nxt[0] = 1'b1;
                end
            end
			default: begin
                p_nxt[WIDTH-1:0] 		= a_abs;
				p_nxt[2*WIDTH-1:WIDTH]	= 0;
            end
        endcase
    end
    always_ff @(posedge clk) begin
        if (rst) begin
            p <= '0;
        end else begin
            p <= p_nxt;
        end
    end
	always_comb
	begin
		c = 0;
		if(isUnsigned)
		begin 
			c = (valid_divu == 1)?p[WIDTH-1:0]:p[2*WIDTH-1:WIDTH];
			if(b == 0 && valid_divu)
			// begin c = ~(WIDTH'b0); end
			begin c = ~0; end
			else if(b == 0 && valid_remu)
			begin c = a; end
		end
		else if(valid_div)
		begin
			c = (isDiv_minus)?(~p[WIDTH-1:0]+1):p[WIDTH-1:0];
			if(b == 0)
			// begin c = ~(WIDTH'b0); end
			begin c = ~0; end
		end
		else if(valid_rem)
		begin
			c = (isRem_minus)?(~p[2*WIDTH-1:WIDTH]+1):p[2*WIDTH-1:WIDTH];
			if(b == 0)
			begin c = a; end
		end
	end
	endmodule

module id
    import defines::*;(
    input  logic                        rst,

    input  logic [`REG_WIDTH-1:0]       input_pc,
    input  logic [`INSTR_WIDTH-1:0]     instr,

    output logic [`REG_WIDTH-1:0]       pc,

    //from regs
    input  logic [`REG_WIDTH-1:0]       rs1data,
    input  logic [`REG_WIDTH-1:0]       rs2data,
    //to regs
    output logic [`REG_ADDR_WIDTH-1:0]  rs1addr,
    output logic [`REG_ADDR_WIDTH-1:0]  rs2addr,

    output logic [`REG_WIDTH-1:0]       rs1_data_o,
    output logic [`REG_WIDTH-1:0]       rs2_data_o,
    
    output logic [`REG_ADDR_WIDTH-1:0]  rdAddr,

    output logic                        memread,
    output logic                        memtoreg,
    output logic                        memwrite,

    output logic                        isNotImm,
    output logic                        regwrite,
    output logic [`REG_WIDTH-1:0]       imm,
    output logic [`ALUOP_WIDTH-1:0]     aluop,
    output logic [2:0]                  funct3,
    output logic [6:0]                  funct7,
    output logic [6:0]                  opcode,

    output logic                  		Iswaitedflag_data,
    output logic                  		Iswaitedflag_addr,
    output logic                  		Iswaitedflag_alu,

	output logic 						csr_we,
	output logic 				        is_mret,
	output logic [11:0]					csr_addr,
	output logic [`REG_WIDTH-1:0]       zimm,

	output logic						illegal_instr,
	input  logic						instr_fetched
	);
	assign      opcode  = instr[6:0];
	logic [4:0] rd      = instr[11:7];
	logic [4:0] rs1     = instr[19:15];    
	logic [4:0] rs2     = instr[24:20];
	assign 		funct3  = instr[14:12];
	assign 		funct7  = instr[31:25];

	assign 		csr_addr= instr[31:20];
	assign 		zimm  	= {{(`REG_WIDTH-5){1'b0}}, instr[19:15]};
	assign 		csr_we 	= (opcode == `INSTR_CSR);
	assign 		is_mret = (instr == 32'b00110000001000000000000001110011);

	always_comb
	begin  
		pc 			= input_pc;
		rs1addr     = `REG_ADDR_WIDTH'h0;
		rs2addr     = `REG_ADDR_WIDTH'h0;
		rs1_data_o  = `REG_WIDTH'h0;
		rs2_data_o  = `REG_WIDTH'h0;
		rdAddr      = `REG_ADDR_WIDTH'h0;

		memread     = 1'b0;
		memtoreg    = 1'b0;
		memwrite    = 1'b0;

		isNotImm    = 1'b0;
		imm         = `REG_WIDTH'h0;
		aluop       = 2'b00;
		regwrite    = 1'b0;

		Iswaitedflag_data= 1'b0;
		Iswaitedflag_addr= 1'b0;
		Iswaitedflag_alu = 1'b0;
		illegal_instr	 = 0;
		case(opcode) 
			`INSTR_R:
				begin
					rs1addr     = rs1;
					rs2addr     = rs2;
					rs1_data_o  = rs1data;
					rs2_data_o  = rs2data;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = `REG_WIDTH'h0;
					aluop       = 2'b10;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
					Iswaitedflag_alu = (funct7[0] == 1'b1)?1'b1:1'b0;
				end
			`INSTR_I:
				begin
					rs1addr     = rs1;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = rs1data;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b1;
					imm         = {{(`REG_WIDTH-12){instr[31]}}, instr[31:20]};
					aluop       = 2'b10;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_B:
				begin
					rs1addr     = rs1;
					rs2addr     = rs2;
					rs1_data_o  = rs1data;
					rs2_data_o  = rs2data;
					rdAddr      = 5'b0;

					memtoreg   = 1'b0;
					memread    = 1'b0;
					memwrite   = 1'b0;
					isNotImm   = 1'b0;
					imm        = {{(`REG_WIDTH-13){instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
					aluop      = 2'b01;
					regwrite   = 1'b0;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end 
			`INSTR_J:
				begin
					rs1addr     = `REG_ADDR_WIDTH'h0;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = `REG_WIDTH'h0;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = {{(`REG_WIDTH-21){instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
					aluop       = 2'b11;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_JR:
				begin
					rs1addr     = rs1;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = rs1data;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = {{(`REG_WIDTH-12){instr[31]}},instr[31:20]};
					aluop       = 2'b11;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_U:
				begin
					rs1addr     = `REG_ADDR_WIDTH'h0;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = `REG_WIDTH'h0;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = {{(`REG_WIDTH-32){instr[31]}}, instr[31:12], 12'b0};
					aluop       = 2'b11;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_UPC:
				begin
					rs1addr     = `REG_ADDR_WIDTH'h0;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = `REG_WIDTH'h0;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = {{(`REG_WIDTH-32){instr[31]}}, instr[31:12], 12'b0};
					aluop       = 2'b11;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_S:
				begin
					rs1addr     = rs1;
					rs2addr     = rs2;
					rs1_data_o  = rs1data;
					rs2_data_o  = rs2data;
					rdAddr      = `REG_ADDR_WIDTH'h0;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b1;
					isNotImm    = 1'b1;
					imm        	= {{(`REG_WIDTH-12){instr[31]}}, instr[31:25], instr[11:7]};
					aluop       = 2'b00;
					regwrite    = 1'b0;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b1;
				end
			`INSTR_IL:
				begin
					rs1addr 	= rs1;
					rs2addr 	= `REG_ADDR_WIDTH'h0;
					rs1_data_o 	= rs1data;
					rs2_data_o 	= `REG_WIDTH'h0;
					rdAddr  	= rd;

					memtoreg   	= 1'b1;
					memread    	= 1'b1;
					memwrite   	= 1'b0;
					isNotImm   	= 1'b1;
					imm        	= {{(`REG_WIDTH-12){instr[31]}}, instr[31:20]};
					aluop      	= 2'b00;
					regwrite   	= 1'b1;
					Iswaitedflag_data= 1'b1;
					Iswaitedflag_addr= 1'b0;
           	 	end
			`INSTR_RW:
				begin
					rs1addr     = rs1;
					rs2addr     = rs2;
					rs1_data_o  = rs1data;
					rs2_data_o  = rs2data;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = `REG_WIDTH'h0;
					aluop       = 2'b10;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
					Iswaitedflag_alu = (funct7[0] == 1'b1)?1'b1:1'b0;
           	 	end
			`INSTR_IW:
				begin
					rs1addr     = rs1;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = rs1data;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b1;
					imm         = {{(`REG_WIDTH-12){instr[31]}}, instr[31:20]};
					aluop       = 2'b10;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_CSR:
				begin
					rs1addr     = rs1;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = rs1data;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					imm         = `REG_WIDTH'h0;
					aluop       = 2'b10;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
					case(funct3)
						`OR:		isNotImm    = 1'b1;
						`AND:		isNotImm    = 1'b1;
						`SLL:		isNotImm    = 1'b0;
						`SRL_SRA:	isNotImm    = 1'b1;
						`SLT:		isNotImm    = 1'b0;
						`SLTU:		isNotImm    = 1'b0;
						default:	isNotImm	= 1'b0;
					endcase
				end
			default:
				begin
					pc = input_pc;
					rs1addr     = `REG_ADDR_WIDTH'h0;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = `REG_WIDTH'h0;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = `REG_ADDR_WIDTH'h0;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = `REG_WIDTH'h0;
					aluop       = 2'b00;
					regwrite    = 1'b0;
					Iswaitedflag_data 	= 1'b0;
					Iswaitedflag_addr	= 1'b0;

					illegal_instr	 	= instr_fetched;
				end
			endcase 
	end
	endmodule 

module ALU
    import defines::*;(
	input logic						clk, rst,
    input logic [`REG_WIDTH-1:0]    pc_i,
    input logic [`REG_WIDTH-1:0]    rs1_data_i,
    input logic [`REG_WIDTH-1:0]    rs2_data_i,
    input logic [2:0]               funct3,
    input logic [6:0]               funct7,
    input logic [`REG_WIDTH-1:0]    imm,
    input logic [6:0]               opcode,

    input logic                     isNotImm,
    input logic [`ALUOP_WIDTH-1:0]  aluop,
    
    output logic                    branchFlag,

    output logic [`REG_WIDTH-1:0]   result,
    output logic [`REG_WIDTH-1:0]   result_ff,
    output logic [`REG_WIDTH-1:0]   pc_o,
	output logic 					done,

    input  logic [`REG_WIDTH-1:0] 	csr_rdata,
    input  logic [`REG_WIDTH-1:0] 	zimm,
    output logic [`REG_WIDTH-1:0] 	csr_wdata,
    output logic [`REG_WIDTH-1:0] 	csr_result
	);

	logic [`REG_WIDTH-1:0] a, b;
	assign a = rs1_data_i;
	assign b = (isNotImm == 1'b0)? rs2_data_i : imm;

	logic  JudgeE, JudgeL, JudgeUL; 
	assign JudgeE = (a == b);
	assign JudgeL = ($signed(a) <  $signed(b));
	assign JudgeUL= (a <  b);
	logic  isMul;
	logic  isMulw;
	logic  isDiv;
	logic  isDivw;
	logic  isDivu;
	logic  isDivuw;
	logic  isRem;
	logic  isRemw;
	logic  isRemu;
	logic  isRemuw;
	logic  valid_w;
	assign valid_w= isDivw|isDivuw|isRemw|isRemuw;
	logic  muldone;
	logic  divdone1;
	logic  divdone2;
	logic  [`REG_WIDTH-1:0] result_mul;
	logic  [`REG_WIDTH-1:0] result_div1;
	logic  [31:0] 			result_div2;

	logic [63:0] tem64;
	logic [31:0] tem32;
	always_comb
	begin
		tem64 		= 'b0;
		tem32 		= 'b0;
		branchFlag  = 1'b0;
		result      = `REG_WIDTH'b0;
		pc_o        = pc_i;
		isMul  		= 0;
		isMulw		= 0;
		isDiv  		= 0;
		isDivw		= 0;
		isDivu 		= 0;
		isDivuw		= 0;
		isRem  		= 0;
		isRemw		= 0;
		isRemu 		= 0;
		isRemuw 	= 0;

		csr_wdata 	= 0;
		csr_result 	= 0;
		case(aluop)
			2'b10:
				begin
					case(funct3)
						`ADD://Actually ADD or SUB or MUL
							begin
								if(opcode == `INSTR_I) // ADDI
								begin 
									result = a + b;
								end
								else if(opcode == `INSTR_IW)  // ADDIW
								begin
									// ADD IMM to 32 signed to 64
									tem64 = a + b;
									tem32 = tem64[31:0];
									result = {{32{tem32[31]}}, tem32};
								end
								else if(opcode == `INSTR_RW && funct7[0] == 1'b0) // ADDW OR SUBW
								begin
									tem64 = (funct7[5] == 1'b0)? (a + b):(a - b);
									tem32 = tem64[31:0];
									result = {{32{tem32[31]}}, tem32};
								end
								else if(opcode == `INSTR_RW && funct7[0] == 1'b1) // MULW
								begin
									isMul = 1;
									isMulw= 1;
								end
								else if(opcode == `INSTR_R && funct7[0] == 1'b0) // ADD OR SUB
								begin 
									result = (funct7[5] == 1'b0)? (a + b):(a - b);
								end
								else if(opcode == `INSTR_R && funct7[0] == 1'b1) // MUL
								begin 
									isMul = 1;
								end
							end
						`XOR://Actually XOR or DIV
							begin
								if(opcode == `INSTR_I || ((funct7[0] == 1'b0) && (opcode == `INSTR_R)))		// XOR or XORI
								begin
									result = a ^ b;
								end
								else if(opcode == `INSTR_R && funct7[0] == 1'b1)		// DIV
								begin
									isDiv = 1;
								end
								else if(opcode == `INSTR_RW && funct7[0] == 1'b1)		// DIVW
								begin
									isDiv = 1;
									isDivw= 1;
								end
							end
						`OR: //Actually OR or REM
							begin 
								if(opcode == `INSTR_I || ((funct7[0] == 1'b0) && (opcode == `INSTR_R)))		// OR
								begin
									result = a | b;
								end
								else if(opcode == `INSTR_R && funct7[0] == 1'b1)	// REM
								begin
									isRem = 1;
								end
								else if(opcode == `INSTR_RW&& funct7[0] == 1'b1)	// REMW
								begin
									isRem = 1;
									isRemw= 1;
								end
								else if(opcode == `INSTR_CSR)		// CSRRSI
								begin
									csr_wdata = csr_rdata | zimm;
									csr_result = csr_rdata;
								end
							end 
						`AND://Actually AND or REMU
							begin 
								if(opcode == `INSTR_I || ((funct7[0] == 1'b0) && (opcode == `INSTR_R)))		// AND
								begin
									result = a & b;
								end
								else if(opcode == `INSTR_R && funct7[0] == 1'b1)	// REMU
								begin
									isRemu = 1;
								end
								else if(opcode == `INSTR_RW&& funct7[0] == 1'b1)	// REMUW
								begin
									isRemu = 1;
									isRemuw= 1;
								end
								else if(opcode == `INSTR_CSR)		// CSRRCI
								begin
									csr_wdata = csr_rdata & ~zimm;
									csr_result = csr_rdata;
								end
							end 
						`SLL://Actually SLL
							begin
								case(opcode)
									`INSTR_R:							// SLL
									begin 
										result = a << b[5:0];
									end
									`INSTR_I: 							// SLLI
									begin 
										result = a << b[5:0];
									end
									`INSTR_RW: 							// SLLW
									begin
										tem64 = a << b[4:0];
										tem32 = tem64[31:0];
										result = {{32{tem32[31]}}, tem32};
									end
									`INSTR_IW: 							// SLLIW
									begin
										if(b[5] == 0)
										begin
											tem64 = a << b[4:0];
											tem32 = tem64[31:0];
											result = {{32{tem32[31]}}, tem32};
										end
									end
									`INSTR_CSR: 						// CSRRW
									begin
										csr_wdata = rs1_data_i;
										csr_result = csr_rdata;
									end
									default:
										result = 'b0;
								endcase
							end
						`SRL_SRA: //Actually SRA or SRL or DIVU
							begin
								if(opcode == `INSTR_CSR)		// CSRRWI
								begin
									csr_wdata = zimm;
									csr_result = csr_rdata;
								end
								else if(funct7[5] == 1'b1) 			// SRA
								begin
									case(opcode)
										`INSTR_R:
										begin
											if(funct7[0] == 0)		// SRA
											begin
												result = (a >> b[5:0]) | (( {64{a[63]}} << (64 - {1'b0, b[5:0]}) ));
											end
										end
										`INSTR_I: 				// SRAI
										begin
											result = (a >> b[5:0]) | (( {64{a[63]}} << (64 - {1'b0, b[5:0]}) ));
										end
										`INSTR_RW: 				// SRAW
										begin
											tem32 = (a[31:0] >> b[4:0]) | (( {32{a[31]}} << (32 - {1'b0, b[4:0]}) ));
											result= {{32{tem32[31]}}, tem32};
										end 
										`INSTR_IW: 				// SRAIW
										begin
											if(b[5] == 0)
											begin
												tem32 = (a[31:0] >> b[4:0]) | (( {32{a[31]}} << (32 - {1'b0, b[4:0]}) ));
												result= {{32{tem32[31]}}, tem32};
											end
										end
										default:
											result = 'b0;
									endcase 
								end
								else 							// SRL or DIVU
								begin  
									case(opcode)
									`INSTR_R:				// SRL or DIVU
									begin
										if(funct7[0] == 0)
										begin
											result = (a >> b[5:0]) & (~( {64{1'b1}} << (64 - {1'b0, b[5:0]}) ));
										end
										else if(funct7[0] == 1) // DIVU
										begin
											isDivu = 1;
										end
									end
									`INSTR_I: 				// SRLI
									begin
										result = (a >> b[5:0]) & (~( {64{1'b1}} << (64 - {1'b0, b[5:0]}) ));
									end
									`INSTR_RW: 				// SRLW or DIVUW
									begin
										if(funct7[0] == 0) 	// SRLW 
										begin
											tem32 = (a[31:0] >> b[4:0]) & (~( {32{1'b1}} << (32 - {1'b0, b[4:0]}) ));
											result= {{32{tem32[31]}}, tem32};
										end
										else if(funct7[0] == 1) // DIVUW
										begin
											isDivu = 1;
											isDivuw = 1;
										end
									end 
									`INSTR_IW: 				// SRLIW
									begin
										if(b[5] == 0)
										begin
											tem32 = (a[31:0] >> b[4:0]) & (~( {32{1'b1}} << (32 - {1'b0, b[4:0]}) ));
											result= {{32{tem32[31]}}, tem32};
										end
									end
									default:
										result = 'b0;
									endcase
								end
							end
						`SLT:
							begin 
								if(opcode == `INSTR_CSR)		// CSRRS
								begin
									csr_wdata = csr_rdata | rs1_data_i;
									csr_result = csr_rdata;
								end
								else
								begin
									result = ($signed(a) < $signed(b))? `REG_WIDTH'b1 : `REG_WIDTH'b0;
								end
							end 
						`SLTU:
							begin 
								if(opcode == `INSTR_CSR)	// CSRRC
								begin
									csr_wdata = csr_rdata & ~rs1_data_i;
									csr_result = csr_rdata;
								end
								else
								begin
									result = (a < b)? `REG_WIDTH'b1 : `REG_WIDTH'b0;
								end
							end 
						default:
							result = 'b0;
					endcase
				end 
			2'b01: // B型指�??
				begin
					case(funct3)
						3'b000: // beq
						begin
							pc_o = (JudgeE == 1'b1)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeE == 1'b1)? 1'b1 : 1'b0;
						end
						3'b001: // bne
						begin
							pc_o = (JudgeE == 1'b0)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeE == 1'b1)? 1'b0 : 1'b1;
						end
						3'b100: // blt
						begin
							pc_o = (JudgeL == 1'b1)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeL == 1'b1)? 1'b1 : 1'b0;
						end
						3'b101: // bge
						begin
							pc_o = (JudgeL == 1'b0)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeL == 1'b1)? 1'b0 : 1'b1;
						end
						3'b110: // bltu
						begin
							pc_o = (JudgeUL == 1'b1)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeUL == 1'b1)? 1'b1 : 1'b0;
						end
						3'b111: // bgeu
						begin
							pc_o = (JudgeUL == 1'b0)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeUL == 1'b1)? 1'b0 : 1'b1;
						end
						default:
							result = 'b0;
					endcase    
				end
			2'b11: // J、JR、U、UPC型指�??
				begin
					case(opcode)
						`INSTR_J:begin 
							result  = pc_i + `REG_WIDTH'h4;
							pc_o    = pc_i + imm;
							branchFlag = 1'b1;
						end
						`INSTR_JR:begin 
							result  = pc_i + `REG_WIDTH'h4;
							pc_o    = a + imm;
							branchFlag = 1'b1;
						end
						`INSTR_U:begin 
							result  = imm;
						end 
						`INSTR_UPC:begin 
							result  = pc_i + imm;
						end
						default:
							result = 'b0;
					endcase
				end
			2'b00: // IL和S型的指令
				begin
					result = a + imm;
				end
			default:
				result = 'b0;
		endcase
	end

	multiplier_multicycle multiplier_multicycle(
		.clk(clk), .rst(rst), .valid(isMul),
		.a(a), .b(b), .c(result_mul),
		.done(muldone)
		);

	divider_multicycle #(64) divider_multicycle1 (
		.clk(clk), .rst(rst), .valid_div(isDiv), .valid_divu(isDivu), .valid_rem(isRem), .valid_remu(isRemu), .valid_w(valid_w),
		.a(a), .b(b), .c(result_div1),
		.done(divdone1)
		);
	divider_multicycle #(32) divider_multicycle2 (
		.clk(clk), .rst(rst), .valid_div(isDiv), .valid_divu(isDivu), .valid_rem(isRem), .valid_remu(isRemu), .valid_w(valid_w),
		.a(a[31:0]), .b(b[31:0]), .c(result_div2),
		.done(divdone2)
		);

	assign done = ((isMul & muldone) | ((isDiv|isDivu|isRem|isRemu) & divdone1 & (~valid_w)) | (divdone2 & valid_w));
	always_comb
	begin
		result_ff = 0;
		if(isMul == 1 && isMulw == 0) begin
			result_ff = result_mul;
		end
		else if(isMul == 1 && isMulw == 1) begin
			result_ff = {{32{result_mul[31]}}, result_mul[31:0]};
		end
		else if(isRemuw|isRemw|isDivw|isDivuw) begin
			result_ff = {{32{result_div2[31]}}, result_div2[31:0]};
		end
		else if(isDiv|isDivu|isRem|isRemu) begin
			result_ff = result_div1;
		end
	end 
	endmodule 

module PC_register 
	import common::*; #(parameter WIDTH = 64) (
    input  logic    		 clk, reset,
    input  logic    		 branchFlag,
	input  logic			 hand_in,
    input  logic [WIDTH-1:0] alu_pc,
    input  logic [WIDTH-1:0] pcnext,
    output logic [WIDTH-1:0] q,

    input  logic    		 is_mret,
    input  logic [WIDTH-1:0] mepc,

    output logic [WIDTH-1:0] pc_reg,

    input  logic			 is_csr,
    input  logic [WIDTH-1:0] csr_pc
    );

	// logic [WIDTH-1:0] q_reg;

    // always_ff @(posedge clk, posedge reset)
    // if (reset)   
	// 	q_reg <= PCINIT;  
	// else if(is_csr)
	// 	q_reg <= csr_pc;
    // else if(hand_in && !is_mret)
	//     q_reg <= ((branchFlag == 1)? alu_pc : pcnext);
	// else if(hand_in && is_mret)
	// 	q_reg <= mepc;
	// else 
	// 	q_reg <= q_reg;
	// assign q = q_reg;

	// logic [WIDTH-1:0] pc_nxt;
    // always_comb
	// begin
	// 	pc_nxt = 0;
	// 	if (reset)   
	// 		pc_nxt = PCINIT;  
	// 	else if(is_csr)
	// 		pc_nxt = csr_pc;
	// 	else if(hand_in && !is_mret)
	// 		pc_nxt = ((branchFlag == 1)? alu_pc : pcnext);
	// 	else if(hand_in && is_mret)
	// 		pc_nxt = mepc;
	// end
	// assign pc_reg = (hand_in == 0)? q : pc_nxt;
	logic [WIDTH-1:0] pc_nxt;
    always_ff @(posedge clk, posedge reset)
    if (reset)   
		q <= PCINIT;  
    else if(hand_in)
	    q <= pc_nxt;
	else if(is_csr)
		q <= csr_pc;
	else 
		q <= q;
	// assign q = q_reg;

    always_comb
	begin
		pc_nxt = 0;
		if (reset)   
			pc_nxt = PCINIT;  
		else if(is_csr)
			pc_nxt = csr_pc;
		else if(!is_mret)
			pc_nxt = ((branchFlag == 1)? alu_pc : pcnext);
		else if(is_mret)
			pc_nxt = mepc;
	end
	assign pc_reg = (hand_in == 0)? q : pc_nxt;

	endmodule

module registerFile
    import defines::*; (
    input  logic clk,
    input  logic rst,
    input  logic we,
	input  logic flag_IL_S,
	input  logic flag_IL_S_done,
	input  logic flag_Alu,
	input  logic flag_Alu_done,
    input  logic memtoreg,
    input  logic [`REG_ADDR_WIDTH-1:0]  writeAddr,
    input  logic [`REG_ADDR_WIDTH-1:0]  readAddr1,
    input  logic [`REG_ADDR_WIDTH-1:0]  readAddr2,

    input  logic [`REG_WIDTH-1:0]       writeData3,
    input  logic [`REG_WIDTH-1:0]       aluout,
    input  logic [`REG_WIDTH-1:0]       aluout_ff,
    output logic [`REG_WIDTH-1:0]       readData1,
    output logic [`REG_WIDTH-1:0]       readData2,
    output reg   [`REG_WIDTH-1:0]       regs_nxt [`REG_COUNT-1:0],

	input  logic [6:0] 					opcode,
	input  logic [2:0] 					funct3,
	input  logic [`REG_WIDTH-1:0] 		csr_result
    );

    reg   [`REG_WIDTH-1:0] regs [`REG_COUNT-1:0]; // 32��64λ�Ĵ���

    always_comb begin
        for (int i = 0; i < `REG_COUNT; i++) begin
            regs_nxt[i] = regs[i];
        end
        if (we && writeAddr != `REG_ADDR_WIDTH'b0) begin
            if((~(flag_IL_S | flag_Alu)) && opcode != `INSTR_CSR) begin
                regs_nxt[writeAddr] = (memtoreg == 1) ? writeData3 : aluout;
            end else if(flag_IL_S & memtoreg & flag_IL_S_done) begin
                case(funct3[2:0])
                    3'b000:  regs_nxt[writeAddr]  = {{56{writeData3[7]}}, writeData3[7:0]};  	// lb
                    3'b100:  regs_nxt[writeAddr]  = {56'b0                , writeData3[7:0]};  	// lbu
                    3'b001:  regs_nxt[writeAddr]  = {{48{writeData3[15]}}, writeData3[15:0]};	// lh
                    3'b101:  regs_nxt[writeAddr]  = {48'b0                , writeData3[15:0]}; 	// lhu
                    3'b010:  regs_nxt[writeAddr]  = {{32{writeData3[31]}}, writeData3[31:0]};	// lw
                    3'b110:  regs_nxt[writeAddr]  = {32'b0                , writeData3[31:0]}; 	// lwu
                    3'b011:  regs_nxt[writeAddr]  = writeData3[63:0];                         	// ld
                    default: regs_nxt[writeAddr]  = writeData3[63:0];
                endcase
            end else if(flag_Alu & flag_Alu_done) begin
                regs_nxt[writeAddr] = aluout_ff;
			end else if(opcode == `INSTR_CSR)
                regs_nxt[writeAddr] = csr_result;
        end
    end

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            for (int i = 0; i < `REG_COUNT; i++) begin
                regs[i] <= `REG_WIDTH'b0;
            end
        end else begin
            for (int i = 0; i < `REG_COUNT; i++) begin
                regs[i] <= regs_nxt[i];
            end
        end
    end

    // ���ݶ�ȡ�߼�
    assign readData1 = (readAddr1 == 0) ? 0 : regs[readAddr1];
    assign readData2 = (readAddr2 == 0) ? 0 : regs[readAddr2];
	endmodule

module reqid_set
    import common::*; import defines::*;(
	input  logic       		 		clk, reset, hand_in, 
	output ibus_req_t  		 		ireq,
	input  ibus_resp_t 		 		iresp,
	output dbus_req_t  		 		dreq,
	input  dbus_resp_t 		 		dresp,

	input  logic					Iswaitedflag_alu,
	input  logic					ALUdone,
	input  logic [`REG_WIDTH-1:0]	pc,

	input  logic					Iswaitedflag_data,
	input  logic					Iswaitedflag_addr,
	input  logic [`REG_WIDTH-1:0]	aluOut,
	input  logic					memWrite,
	input  logic [2:0]				funct3,
	input  logic [`REG_WIDTH-1:0]	writeData,

	input  satp_t					csr_satp,
	input  logic [1:0]				mode,

	output logic 					trans_needed,

	input  logic [`REG_WIDTH-1:0]   pc_reg,
	input  logic					is_csr_immediately
	);

	logic [11:0] i_page_table_index1 = {3'b0, pc_reg[38:30]};
	logic [11:0] i_page_table_index2 = {3'b0, pc_reg[29:21]};
	logic [11:0] i_page_table_index3 = {3'b0, pc_reg[20:12]};

	logic [1:0] dreq_access_count;

	logic [`REG_WIDTH-1:0] dresp_data1_reg;
	logic [`REG_WIDTH-1:0] dresp_data2_reg;
	
	logic [`REG_WIDTH-1:0] dresp_data;
	logic [`REG_WIDTH-1:0] dresp_data_reg;
	always @(posedge clk, posedge reset)
	begin
		if(reset)
		begin
			dreq_access_count <= 0;
			dresp_data1_reg <= 0;
			dresp_data2_reg <= 0;
			dresp_data_reg <= 0;
		end
		else if(hand_in)
		begin
			dreq_access_count <= 0;
			dresp_data1_reg <= 0;
			dresp_data2_reg <= 0;
			dresp_data_reg <= 0;
		end
		else if(dresp.data_ok && dreq_access_count == 0)
		begin
			dreq_access_count <= 1;
			dresp_data1_reg <= dresp.data;
			dresp_data_reg <= dresp.data;
		end
		else if(dresp.data_ok && dreq_access_count == 1)
		begin
			dreq_access_count <= 2;
			dresp_data2_reg <= dresp.data;
		end
		else if(dresp.data_ok && dreq_access_count == 2)
		begin
			dreq_access_count <= 3;
			dresp_data_reg <= dresp.data;
		end
		else
		begin
			dreq_access_count <= dreq_access_count;
			dresp_data1_reg <= dresp_data1_reg;
			dresp_data2_reg <= dresp_data2_reg;
			dresp_data_reg <= dresp_data_reg;
		end
	end
	assign dresp_data  = (dresp.data_ok == 0)? dresp_data_reg : dresp.data;
	
	logic ff;
	logic [`REG_WIDTH-1:0] ff_addr;
	logic ireq_sync_1, ireq_sync_2;
	always @(posedge clk, posedge reset)
	begin
		if(Iswaitedflag_alu && (ALUdone == 0))
		begin ff <= 0; end
		else
		begin ff <= 1;end

		if(reset)
		begin
			ff_addr <= PCINIT;
		end
		else if (trans_needed == 0) 
		begin
			ff_addr <= pc_reg;
		end 
		else 
		begin
			ff_addr <= {8'b0, dresp_data[53:10], pc[11:0]};
		end
	end
	assign ireq.valid 	= ff;
	assign ireq.addr 	= ff_addr;

	assign trans_needed = (mode != 3) && (csr_satp.mode == 8);
	// assign trans_needed = 0;

	assign dreq.valid	= (Iswaitedflag_data|Iswaitedflag_addr|(trans_needed && dreq_access_count != 2'b11) )& !is_csr_immediately;
	always_comb
	begin
		if((trans_needed && dreq_access_count != 2'b11))
		begin
			case(dreq_access_count)
				0: dreq.addr = {8'b0, csr_satp.ppn, 12'b0} + {52'b0, (i_page_table_index1 << 3)};
				1: dreq.addr = {8'b0, dresp_data1_reg[53:10], 12'b0} + {52'b0, (i_page_table_index2 << 3)};
				2: dreq.addr = {8'b0, dresp_data2_reg[53:10], 12'b0} + {52'b0, (i_page_table_index3 << 3)};
				default:dreq.addr = 0;
			endcase
			dreq.size = MSIZE8;
			dreq.strobe = 8'b0;
			dreq.data = 0;
		end
		else if(memWrite == 1)
		begin
			dreq.addr = aluOut;
			case(funct3[1:0])
					2'b00: dreq.size = MSIZE1;
					2'b01: dreq.size = MSIZE2;
					2'b10: dreq.size = MSIZE4;
					2'b11: dreq.size = MSIZE8;
					default: dreq.size = MSIZE8;
			endcase
			case(funct3[1:0])
				2'b00: 	// sb
					begin 
						case(aluOut[3:0]%8)
							0: begin dreq.strobe = 8'b00000001; dreq.data = writeData; end 
							1: begin dreq.strobe = 8'b00000010; dreq.data = writeData << 8; end 
							2: begin dreq.strobe = 8'b00000100; dreq.data = writeData << 16; end 
							3: begin dreq.strobe = 8'b00001000; dreq.data = writeData << 24; end 
							4: begin dreq.strobe = 8'b00010000; dreq.data = writeData << 32; end 
							5: begin dreq.strobe = 8'b00100000; dreq.data = writeData << 40; end 
							6: begin dreq.strobe = 8'b01000000; dreq.data = writeData << 48; end 
							7: begin dreq.strobe = 8'b10000000; dreq.data = writeData << 56; end 
							default: begin dreq.strobe = 8'b00000001; dreq.data = writeData; end 
						endcase
					end
				2'b01:  // sh
					begin 
						case(aluOut[3:0]%8)
							0: begin dreq.strobe = 8'b00000011; dreq.data = writeData; end 
							1: begin dreq.strobe = 8'b00000110; dreq.data = writeData << 8; end 
							2: begin dreq.strobe = 8'b00001100; dreq.data = writeData << 16; end 
							3: begin dreq.strobe = 8'b00011000; dreq.data = writeData << 24; end 
							4: begin dreq.strobe = 8'b00110000; dreq.data = writeData << 32; end 
							5: begin dreq.strobe = 8'b01100000; dreq.data = writeData << 40; end 
							6: begin dreq.strobe = 8'b11000000; dreq.data = writeData << 48; end 
							7: begin dreq.strobe = 8'b10000000; dreq.data = writeData << 56; end 
							default: begin dreq.strobe = 8'b00000011; dreq.data = writeData; end 
						endcase
					end
				2'b10:  // sw
					begin 
						case(aluOut[3:0]%8)
							0: begin dreq.strobe = 8'b00001111; dreq.data = writeData; end 
							1: begin dreq.strobe = 8'b00011110; dreq.data = writeData << 8; end 
							2: begin dreq.strobe = 8'b00111100; dreq.data = writeData << 16; end 
							3: begin dreq.strobe = 8'b01111000; dreq.data = writeData << 24; end 
							4: begin dreq.strobe = 8'b11110000; dreq.data = writeData << 32; end 
							5: begin dreq.strobe = 8'b11100000; dreq.data = writeData << 40; end 
							6: begin dreq.strobe = 8'b11000000; dreq.data = writeData << 48; end 
							7: begin dreq.strobe = 8'b10000000; dreq.data = writeData << 56; end 
							default: begin dreq.strobe = 8'b00001111; dreq.data = writeData; end 
						endcase
					end
				2'b11:  // sd
					begin 
						case(aluOut[3:0]%8)
							0: begin dreq.strobe = 8'b11111111; dreq.data = writeData; end 
							1: begin dreq.strobe = 8'b11111110; dreq.data = writeData << 8; end 
							2: begin dreq.strobe = 8'b11111100; dreq.data = writeData << 16; end 
							3: begin dreq.strobe = 8'b11111000; dreq.data = writeData << 24; end 
							4: begin dreq.strobe = 8'b11110000; dreq.data = writeData << 32; end 
							5: begin dreq.strobe = 8'b11100000; dreq.data = writeData << 40; end 
							6: begin dreq.strobe = 8'b11000000; dreq.data = writeData << 48; end 
							7: begin dreq.strobe = 8'b10000000; dreq.data = writeData << 56; end 
							default: begin dreq.strobe = 8'b11111111; dreq.data = writeData; end 
						endcase
					end
				default:	begin dreq.strobe = 8'b11111111; dreq.data = writeData; end 
			endcase
		end
		else
			begin dreq.addr = aluOut; dreq.strobe = 8'b00000000; dreq.data = 64'b0; dreq.size = MSIZE1; end
	end

	endmodule

module core import common::*; import defines::*;(
	input  logic       		 	clk, reset,
	output ibus_req_t  		 	ireq,
	input  ibus_resp_t 		 	iresp,
	output dbus_req_t  		 	dreq,
	input  dbus_resp_t 		 	dresp,
	input  logic       		 	trint, swint, exint
	);
	/* TODO: Add your CPU-Core here. */
	
	logic 					 	hand_in;

    logic [`INSTR_WIDTH-1:0] 	instr;
    logic [`REG_WIDTH-1:0]   	readData;
	logic [`REG_WIDTH-1:0]	 	writeData;
	assign						readData = dresp.data;

    logic                    	memWrite;
    logic                    	memRead;
	logic						Iswaitedflag_data;	
	logic						Iswaitedflag_addr;	
	logic						Iswaitedflag_alu;	
    logic						memtoreg;
    logic						isNotImm;
    logic						regwrite;
    logic						branchFlag;
    logic [`ALUOP_WIDTH-1:0]	aluop;
    logic [2:0]					funct3;
    logic [6:0]					funct7;
    logic [`REG_WIDTH-1:0]      imm;
    logic [6:0]                 opcode;

    logic [`REG_WIDTH-1:0]   	aluOut;
    logic [`REG_WIDTH-1:0]   	aluOut_ff;
	logic 						ALUdone;


	logic 						is_mret;

    logic [`REG_WIDTH-1:0]	 	q;
    logic [`REG_WIDTH-1:0]	 	pc;	assign pc = q;
    logic [`REG_WIDTH-1:0]      pcnext;
    logic [`REG_WIDTH-1:0]      id_pc;
    logic [`REG_WIDTH-1:0]      alu_pc;
    logic [`REG_WIDTH-1:0]      pc_reg;
    logic [`REG_WIDTH-1:0]      csr_pc;

	reg [`REG_WIDTH-1:0] 		regs_nxt 	[`REG_COUNT-1:0];
	reg [`INSTR_WIDTH-1:0] 		instr_g;

    logic [`REG_WIDTH-1:0] 		rD1, rD2;
	logic [`REG_ADDR_WIDTH-1:0] rA1, rA2, rdA;
    logic [`REG_WIDTH-1:0] 		id_rD1, id_rD2;
    assign 						writeData = id_rD2;

	// csr
    CSR_reg 					csr_reg;
	logic [11:0]				csr_addr;
	logic 						csr_we;
    logic [`REG_WIDTH-1:0] 		csr_wdata;
    logic [`REG_WIDTH-1:0] 		csr_rdata;
    logic [`REG_WIDTH-1:0] 		csr_result;
	logic [`REG_WIDTH-1:0]  	zimm;
	logic 						trans_needed;

	// judge exception_pending | interrupt_pending
	logic 						instr_unaligned; 
	logic 						data_unaligned_r;
	logic 						data_unaligned_w; 

	logic [3:0] 				size;
	always_comb
		case(funct3[1:0])
			2'b00: size = 4'b0001;
			2'b01: size = 4'b0010;
			2'b10: size = 4'b0100;
			2'b11: size = 4'b1000;
			default: size = 4'b0001;
		endcase
	assign						instr_unaligned  = (pc[1:0] != 2'b00);
	assign						data_unaligned_r = !trans_needed & !memWrite & (aluOut%{60'b0, size} != 0) &(Iswaitedflag_addr | Iswaitedflag_data);
	assign						data_unaligned_w = !trans_needed &  memWrite & (aluOut%{60'b0, size} != 0) &(Iswaitedflag_addr | Iswaitedflag_data);
	logic 						ecall;
	assign						ecall = (instr == 32'b001110011);

	logic 						illegal_instr;	// id
	logic 						instr_fetched;	// id
	logic 						iresp_data_ok_reg;
	always @(posedge clk)
	begin
		if(reset)				iresp_data_ok_reg <= 0;
		else if(hand_in)		iresp_data_ok_reg <= 0;
		else if(iresp.data_ok)	iresp_data_ok_reg <= 1;
	end
	assign						instr_fetched = (iresp.data_ok == 1)? 1:iresp_data_ok_reg;

	logic 						is_csr;
	logic 						is_csr_immediately;
	assign						is_csr_immediately = instr_unaligned|data_unaligned_r|data_unaligned_w|illegal_instr|ecall|trint|swint|exint;

	logic flag;
	assign flag = (pc == 64'h0080001cc4);
	always_comb
	begin
		hand_in = 0;
		if(is_csr_immediately)	
			hand_in = is_csr;
		else if(!trans_needed)
		begin
			if((~(Iswaitedflag_data|Iswaitedflag_addr|Iswaitedflag_alu)))
				hand_in = iresp.data_ok;
			else
				hand_in = dresp.data_ok;
		end
		else if(trans_needed)
				hand_in = iresp.data_ok;
	end
	// logic hand_in1;
	// logic hand_in2;
	// always @(posedge clk, posedge reset)
	// begin
	// 	if(reset) 
	// 	begin
	// 		hand_in1 <= 0;
	// 	end
	// 	else if(is_csr_immediately)	
	// 		hand_in1 <= is_csr;
	// end
	// always_comb
	// begin
	// 	hand_in2 = 0;
	// 	if((!trans_needed))
	// 	begin
	// 		if((~(Iswaitedflag_data|Iswaitedflag_addr|Iswaitedflag_alu)))
	// 			hand_in2 = iresp.data_ok;
	// 		else
	// 			hand_in2 = dresp.data_ok;
	// 	end
	// 	else if(trans_needed)
	// 			hand_in2 = iresp.data_ok;
	// end
	// assign hand_in = (is_csr_immediately==1)?hand_in1:hand_in2;
	

	reqid_set 	reqid_set(
		.clk(clk),
		.reset(reset),
		.hand_in(hand_in),
		.ireq(ireq),
		.iresp(iresp),
		.dreq(dreq),
		.dresp(dresp),
		
		.Iswaitedflag_alu(Iswaitedflag_alu),
		.ALUdone(ALUdone),
		.pc(pc),

		.Iswaitedflag_data(Iswaitedflag_data),
		.Iswaitedflag_addr(Iswaitedflag_addr),
		.aluOut(aluOut),
		.memWrite(memWrite),
		.funct3(funct3),
		.writeData(writeData),

		.csr_satp(csr_reg.satp),
		.mode(csr_reg.mode),

		.trans_needed(trans_needed),
		.pc_reg(pc_reg),

		.is_csr_immediately(is_csr_immediately)
	);

    PC_register pcReg(
        .clk(clk), 
        .reset(reset), 
		.hand_in(hand_in),
        .branchFlag(branchFlag),
        .alu_pc(alu_pc),
        .pcnext(pcnext), 
        .q(q),

		.is_mret(is_mret),
		.mepc(csr_reg.mepc),

		.pc_reg(pc_reg),
		.is_csr(is_csr),
		.csr_pc(csr_pc)
    );

    adder       pcAdd(
        q, 
        `REG_WIDTH'h4, 
        pcnext
    );


	instr_reg 		instr_reg(
		.clk(clk),
		.rst(reset),
		.hand_in(hand_in),
		.iresp_data(iresp.data),
		.iresp_dataok(iresp.data_ok),
		.instr(instr),
		.instr_reg(instr_g)
	);


    id          id(
        .rst(reset),
        .input_pc(pc),
        .instr(instr),
        .pc(id_pc),
        .rs1data(rD1),
        .rs2data(rD2),
        .rs1addr(rA1),
        .rs2addr(rA2),
        .rs1_data_o(id_rD1),
        .rs2_data_o(id_rD2),
        .rdAddr(rdA),
        .memread(memRead),.memtoreg(memtoreg),.memwrite(memWrite),.isNotImm(isNotImm),.regwrite(regwrite),.aluop(aluop),
		.funct3(funct3),.funct7(funct7),.imm(imm),.opcode(opcode),
		.Iswaitedflag_data(Iswaitedflag_data),
		.Iswaitedflag_addr(Iswaitedflag_addr),
		.Iswaitedflag_alu(Iswaitedflag_alu),

		.csr_we(csr_we),
		.is_mret(is_mret),
		.csr_addr(csr_addr),
		.zimm(zimm),

		.illegal_instr(illegal_instr),
		.instr_fetched(instr_fetched)
    );


	csr 		csr(
		.clk(clk),
		.rst(reset),
		.csr_we(csr_we),
		.csr_addr(csr_addr),
		.csr_wdata(csr_wdata),
		.csr_rdata(csr_rdata),
		.csr_reg(csr_reg),

		.is_mret(is_mret),
		.pc(pc),
		.csr_pc(csr_pc),
		.instr_unaligned(instr_unaligned), .data_unaligned_r(data_unaligned_r), .data_unaligned_w(data_unaligned_w), .illegal_instr(illegal_instr), .ecall(ecall),
		.trint(trint), .swint(swint), .exint(exint),
		.is_csr(is_csr),

		.hand_in(hand_in)
	);

    ALU         alu(
		.clk(clk), .rst(reset),
        .pc_i(id_pc),
        .rs1_data_i(id_rD1),.rs2_data_i(id_rD2),
        .funct3(funct3),
        .funct7(funct7),
        .imm(imm),
        .opcode(opcode),
        .isNotImm(isNotImm),
        .aluop(aluop),
        .branchFlag(branchFlag),
        .result (aluOut),
		.result_ff(aluOut_ff),
        .pc_o(alu_pc),
		.done(ALUdone),
		.csr_rdata(csr_rdata), .zimm(zimm), .csr_wdata(csr_wdata), .csr_result(csr_result)
    );    

    registerFile rfile(
        .clk(clk), 
        .rst(reset),
        .we(regwrite),
		.flag_IL_S(Iswaitedflag_data|Iswaitedflag_addr),
		.flag_IL_S_done(dresp.data_ok),
		.flag_Alu(Iswaitedflag_alu),
		.flag_Alu_done(ALUdone),
        .memtoreg(memtoreg),
        .writeAddr(rdA),
        .aluout(aluOut),
        .aluout_ff(aluOut_ff),
        .readAddr1(rA1),
        .readAddr2(rA2),
        .writeData3(readData>>((aluOut[3:0]%8)*8)),
        .readData1(rD1),
        .readData2(rD2),
		.regs_nxt(regs_nxt),
		.opcode(opcode),
		.funct3(funct3),
		.csr_result(csr_result)
    );



`ifdef VERILATOR
	DifftestInstrCommit DifftestInstrCommit(
		.clock              (clk),
		.coreid             (0),			// 无需改动
		.index              (0),			// 多发射时，例化�?�个该模块。前四个 Lab 无需改动�??
		.valid              (hand_in),		// 无提交（或提交的指令是flush导致的bubble时，�??0�??
		.pc                 (pc),			// 这条指令�?? pc
		.instr              (instr),		// 这条指令的内容，�?不改�?
		.skip               (dresp.data_ok&aluOut[31]==0),			// 提交的是�??条内存�?�写指令，且这部分内存属于�?��?�（addr[31] == 0）时，skip �?? 1
		.isRVC              (0),			// 前四�?? Lab 无需改动
		.scFailed           (0),			// 前四�?? Lab 无需改动
		.wen                (regwrite),		// 这条指令�?否写入通用寄存�?�?1 bit
		.wdest              ({3'b0, rdA}),	// 写入�?�?通用寄存�??
		.wdata              (regs_nxt[rdA])	// 写入的�??
	);

	DifftestArchIntRegState DifftestArchIntRegState (
		.clock              (clk),
		.coreid             (0),
		.gpr_0              (regs_nxt[0]),
		.gpr_1              (regs_nxt[1]),
		.gpr_2              (regs_nxt[2]),
		.gpr_3              (regs_nxt[3]),
		.gpr_4              (regs_nxt[4]),
		.gpr_5              (regs_nxt[5]),
		.gpr_6              (regs_nxt[6]),
		.gpr_7              (regs_nxt[7]),
		.gpr_8              (regs_nxt[8]),
		.gpr_9              (regs_nxt[9]),
		.gpr_10             (regs_nxt[10]),
		.gpr_11             (regs_nxt[11]),
		.gpr_12             (regs_nxt[12]),
		.gpr_13             (regs_nxt[13]),
		.gpr_14             (regs_nxt[14]),
		.gpr_15             (regs_nxt[15]),
		.gpr_16             (regs_nxt[16]),
		.gpr_17             (regs_nxt[17]),
		.gpr_18             (regs_nxt[18]),
		.gpr_19             (regs_nxt[19]),
		.gpr_20             (regs_nxt[20]),
		.gpr_21             (regs_nxt[21]),
		.gpr_22             (regs_nxt[22]),
		.gpr_23             (regs_nxt[23]),
		.gpr_24             (regs_nxt[24]),
		.gpr_25             (regs_nxt[25]),
		.gpr_26             (regs_nxt[26]),
		.gpr_27             (regs_nxt[27]),
		.gpr_28             (regs_nxt[28]),
		.gpr_29             (regs_nxt[29]),
		.gpr_30             (regs_nxt[30]),
		.gpr_31             (regs_nxt[31])
	);

	// （暂时不用�?��?
    DifftestTrapEvent DifftestTrapEvent(
		.clock              (clk),
		.coreid             (0),
		.valid              (0),
		.code               (0),
		.pc                 (0),
		.cycleCnt           (0),
		.instrCnt           (0)
	);

	DifftestCSRState DifftestCSRState(
		.clock              (clk),
		.coreid             (0),
		.priviledgeMode     (csr_reg.mode),
		.mstatus            (csr_reg.mstatus),
		.sstatus            (csr_reg.mstatus & 64'h800000030001e000),
		.mepc               (csr_reg.mepc),
		.sepc               (0),
		.mtval              (csr_reg.mtval),
		.stval              (0),
		.mtvec              (csr_reg.mtvec),
		.stvec              (0),
		.mcause             (csr_reg.mcause),
		.scause             (0),
		.satp               (csr_reg.satp),
		.mip                (csr_reg.mip),
		.mie                (csr_reg.mie),
		.mscratch           (csr_reg.mscratch),
		.sscratch           (0),
		.mideleg            (0),
		.medeleg            (0)
	);
`endif
endmodule
`endif