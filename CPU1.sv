module CPU1
(
	input  logic        new_clock,
	output logic [17:0] LEDR,
	output logic [7:0]  LEDG,
	input  logic [17:0] SW,
	output logic [7:0]  HEX0,
	output logic [7:0]  HEX1,
	output logic [7:0]  HEX2,
	output logic [7:0]  HEX3,
	output logic [7:0]  HEX4,
	output logic [7:0]  HEX5,
	output logic [7:0]  HEX6,
	output logic [7:0]  HEX7,
	output logic 		  	SRAMGW,
	output logic 		  	SRAMOE,
	output logic [5:0] 	memory_address_out, // SSRAM R/W address
	input  logic [31:0] SRAM_RD, // SRAM Read data
	output logic [31:0] SRAM_WD,  // SRAM Write data
	output logic [31:0] CacheRD,
	//output logic [31:0] CacheWD,
	input  logic 		  	StallMe,
	//output logic 		  	RAMSTALL,
	input  logic [31:0] ControlWriteData,
	output logic        BlockStatus,
	input  logic [5:0] 	Ad2,
	output logic [31:0] Ad2Data,
	output logic [1:0]  Ad2Tag,
	output logic        Ad2BlockStatus,
	input  logic [5:0] 	Ad3,
	output logic [31:0] Ad3Data,
	output logic [1:0]  Ad3Tag,
	output logic        Ad3BlockStatus,
	input  logic [5:0] 	Ad4,
	output logic [31:0] Ad4Data,
	output logic [1:0]  Ad4Tag,
	output logic        Ad4BlockStatus,
	output logic 		    MemoryWE,
	output logic        MemoryFunct,
	output logic        Hit,
	input  logic [2:0]  FetchBlock,
	input  logic [2:0]  Invalidate
); 
// Block register variables
// ---------------------------------------
	logic [47:0] fetch_in, fetch_out;
	logic [143:0] decode_in, decode_out;
	logic [92:0] execute_in, execute_out;
	logic [71:0] memory_in, memory_out;
	logic [1:0] forward_AE, forward_BE;
	logic stall_PC, stall_FD, clear_FD, clear_DE, forward_AD, forward_BD;
	
// --------------------------------------- //
// --------------------------------------- //
// FETCH BLOCK                			    //
// --------------------------------------- //
// --------------------------------------- //
	
// Program counter
// ---------------------------------------
	logic branch_out;
	logic [15:0] PC, PC1, PC_jump_imm;
	logic [31:0] reg_WD, reg_RD1, reg_RD2, data_out;
	logic [10:0] type_out;
	logic PC_jump_enable;
	logic [31:0] Instruction_out;
	assign PC_jump_enable = type_out[2] | SW[17];
	logic STALLPC;
	assign STALLPC = /*~RAMSTALL &*/ ~stall_PC & ~StallMe;
	assign PC_jump_imm = SW[17] ? 16'd0 : (fetch_out[26] ? reg_RD2[15:0] : fetch_out[15:0]);
	Program_counter pc(branch_out, PC_jump_enable, fetch_out[25:10], PC_jump_imm, new_clock, STALLPC, PC, PC1);
	logic [15:0] PC_out;
	Register #(16) program_address_reg(PC, new_clock, STALLPC, PC_out);
	assign LEDG[7:0] = PC_out[7:0];
	
// ROM
// ---------------------------------------	
	CPU1ROM #(16, 32) instructions(PC_out, Instruction_out);
	
// Fetch register
// ---------------------------------------
	assign fetch_in[31:0] = clear_FD ? 32'd0 : Instruction_out;
	assign fetch_in[47:32] = clear_FD ? 16'd0 : PC;
	logic STALLFD;
	assign STALLFD = (/*~RAMSTALL & */~stall_FD & ~StallMe) ? 1'b1 : 1'b0;
	Register #(48) fetch(fetch_in, new_clock, STALLFD, fetch_out);

// --------------------------------------- //
// --------------------------------------- //
// DECODE BLOCK                			    //
// --------------------------------------- //
// --------------------------------------- //	

// Control decoder
// ---------------------------------------
	Control_Decoder control_decoder(fetch_out[31:0], type_out);
	assign LEDR[10:0] = type_out;
	
// Register file
// ---------------------------------------
	logic [5:0] reg_WA;
	logic [31:0] mul_div_out;
	logic [31:0] shifter_out;
	logic [31:0] alu_out;
	logic [31:0] mem_out;
	logic [8:0] REG_DIS_OUT;
	logic [31:0] display_data;
	logic [31:0] imm_out;
	logic memory_WE;
	// This is used to decode the switches for register content display on the HEX
	Priority_core #(8) decoder(SW[7:0], REG_DIS_OUT);
	
	assign reg_WD = /* type_out[2] ? {16'd0, fetch_out[47:32]} : ( */memory_out[70] ? memory_out[63:32] : memory_out[31:0]/* ) */;
	assign reg_WA = type_out[5] ? fetch_out[14:10] : fetch_out[4:0];
	logic [31:0] localRD1, localRD2;
	
	logic reg_WE;
	assign reg_WE = (type_out[4] & ~memory_WE) | type_out[6] | (type_out[5] & type_out[7]) | 
		(type_out[5] & type_out[8] & 
		((~fetch_out[29] & ~fetch_out[28] & ~fetch_out[27] & ~fetch_out[26]) | 
		(~fetch_out[29] & ~fetch_out[28] & fetch_out[27] & ~fetch_out[26]))) | 
		(type_out[5] & type_out[10]);
	
	Register_File #(32) register_file(new_clock, memory_out[64], memory_out[69:65], fetch_out[9:5], 
		fetch_out[4:0], reg_WD, localRD1, localRD2, REG_DIS_OUT, display_data);

	assign reg_RD1 = ((memory_out[69:65] == fetch_out[9:5]) & memory_out[64] & (fetch_out[9:5] != 5'd0)) ? reg_WD : localRD1;
	assign reg_RD2 = ((memory_out[69:65] == fetch_out[4:0]) & memory_out[64] & (fetch_out[4:0] != 5'd0)) ? reg_WD : localRD2;
	
// Immediate
// ---------------------------------------
	Immediate #(32) immediate(fetch_out[23:10], type_out[6], fetch_out[28:26], fetch_out[25:24], imm_out);
		
// Branch
// ---------------------------------------
	logic [31:0] branchRSData, branchRTData;
	assign branchRSData = forward_AD ? execute_out[31:0] : reg_RD1;
	assign branchRTData = forward_BD ? execute_out[31:0] : reg_RD2;
	Branch #(32) branch(type_out[1], type_out[3], fetch_out[27:26], branchRTData, branchRSData, branch_out);
	assign memory_WE = (type_out[4] & ((fetch_out[26] & ~fetch_out[27] & fetch_out[28]) 
		| (fetch_out[27] & fetch_out[28]))) ? 1'b1 : 1'b0;
	
// Decode register
// ---------------------------------------
	assign decode_in[10:0] = clear_DE ? 11'd0 : type_out;
	assign decode_in[42:11] = clear_DE ? 32'd0 : reg_RD1;
	assign decode_in[74:43] = clear_DE ? 32'd0 : reg_RD2;
	assign decode_in[79:75] = clear_DE ? 5'd0 : reg_WA;
	assign decode_in[80] = clear_DE ? 1'd0 : reg_WE;
	assign decode_in[112:81] = clear_DE ? 32'd0 : imm_out;
	assign decode_in[116:113] = clear_DE ? 4'd0 : fetch_out[29:26]; // function codes
	assign decode_in[132:117] = clear_DE ? 16'd0 : fetch_out[47:32]; // PC1
	assign decode_in[137:133] = clear_DE ? 5'd0 : fetch_out[9:5]; // rs
	assign decode_in[142:138] = clear_DE ? 5'd0 : fetch_out[4:0]; // rt
	assign decode_in[143] = clear_DE ? 1'd0 : memory_WE;
	Register #(144) decode(decode_in, new_clock, ~StallMe, decode_out);
	
// --------------------------------------- //
// --------------------------------------- //
// EXECUTE BLOCK				                //
// --------------------------------------- //
// --------------------------------------- //

	logic [31:0] A, B, Bpre, immediate_value;
	logic [3:0] funct;
	logic LUI_EN, BRANCHLTZ_EN, JUMP_EN, BRANCH_EN, MEM_EN, RTYPE_EN, IMM_EN, ALU_EN, MULDIV_EN, DELAY_EN, SHIFT_EN;
	assign immediate_value = decode_out[112:81];
	assign funct = decode_out[116:113];
	assign LUI_EN = decode_out[0];
	assign BRANCHLTZ_EN = decode_out[1];
	assign JUMP_EN = decode_out[2];
	assign BRANCH_EN = decode_out[3];
	assign MEM_EN = decode_out[4];
	assign RTYPE_EN = decode_out[5];
	assign IMM_EN = decode_out[6];
	assign ALU_EN = (decode_out[5] & decode_out[7]);
	assign MULDIV_EN = (decode_out[5] & decode_out[8]);
	assign DELAY_EN = (decode_out[5] & decode_out[9]);
	assign SHIFT_EN = (decode_out[5] & decode_out[10]);
	
	logic [31:0] forward_AE_mux_in[3:0];
	assign forward_AE_mux_in[0] = decode_out[42:11];
	assign forward_AE_mux_in[1] = memory_out[71] ? memory_out[63:32] : memory_out[31:0];
	assign forward_AE_mux_in[2] = execute_out[31:0];
	assign forward_AE_mux_in[3] = execute_out[31:0];
	MUX #(2, 32) forward_AE_mux(forward_AE, forward_AE_mux_in, A);
	
	logic [31:0] forward_BE_mux_in[3:0];
	assign forward_BE_mux_in[0] = decode_out[74:43];
	assign forward_BE_mux_in[1] = memory_out[71] ? memory_out[63:32] : memory_out[31:0];
	assign forward_BE_mux_in[2] = execute_out[31:0];
	assign forward_BE_mux_in[3] = execute_out[31:0];
	MUX #(2, 32) forward_BE_mux(forward_BE, forward_BE_mux_in, Bpre);
	assign B = (~RTYPE_EN | MEM_EN) ? immediate_value : Bpre;
	logic MemtoReg;
	assign MemtoReg = (MEM_EN & ~decode_out[143]);
	
// Shifter
// ---------------------------------------
	Shifter shifter(B, decode_out[9:5],	A, funct[2:0], shifter_out);

// Mul/Div
// ---------------------------------------
	MUL_DIV #(32) mul_div(A, B, funct, new_clock, mul_div_out);
	
// ALU
// ---------------------------------------
	logic overflow, carry_out; 
	logic [3:0] aluF;
	assign aluF = MEM_EN ? 4'd0 : (IMM_EN ? ({{~funct[3] & funct[2]}, funct[2:0]}) : funct);
	ALU #(32) alu(A, B, aluF, alu_out, overflow, carry_out);
	
	logic [31:0] Y;
	assign Y = SHIFT_EN ? shifter_out : (MULDIV_EN ? mul_div_out : alu_out);
	
// Execute register
// ---------------------------------------
	assign execute_in[31:0]  = Y;
	assign execute_in[48:32] = decode_out[132:117]; // PC1
	assign execute_in[49]    = decode_out[80]; // Reg_WE
	assign execute_in[54:50] = decode_out[79:75]; // reg_WA
	assign execute_in[55]    = decode_out[143]; // memory_WE
	assign execute_in[87:56] = Bpre;
	assign execute_in[88]    = MEM_EN;
	assign execute_in[91:89] = funct[2:0];
	assign execute_in[92] = MemtoReg;
	Register #(93) execute(execute_in, new_clock, ~StallMe, execute_out);
	logic [5:0] memory_address_in;
	assign memory_address_in = Y[5:0];
	logic memAddEn;
	assign memAddEn = MEM_EN & ~StallMe;
	Register #(6) memory_address(memory_address_in, new_clock, memAddEn, memory_address_out);
	
// --------------------------------------- //
// --------------------------------------- //
// MEMORY BLOCK	                         //
// --------------------------------------- //
// --------------------------------------- //

// Memory
// ---------------------------------------

	// Contains the L1 cache
	logic [2:0] memF;
	assign memF = execute_out[88] ? execute_out[91:89] : 3'd0;
		Memory_interface #(32) memory_interface(execute_out[88], execute_out[87:56], memF, memory_address_out, 
		new_clock, mem_out, MemEn, SRAM_RD, SRAM_WD,  SRAMGW, SRAMOE, StallMe, CacheRD,
		ControlWriteData, BlockStatus, Ad2, Ad2Data, Ad2Tag, Ad2BlockStatus, Ad3, Ad3Data, Ad3Tag, Ad3BlockStatus, 
		Ad4, Ad4Data, Ad4Tag, Ad4BlockStatus, Hit, FetchBlock, Invalidate);
	
	assign MemoryWE = execute_out[55];
	assign MemoryFunct = execute_out[88];
	
// Memory register
// ---------------------------------------
	assign memory_in[31:0] = execute_out[31:0]; // Y
	assign memory_in[63:32] = mem_out;
	assign memory_in[64] = execute_out[49]; // reg_WE
	assign memory_in[69:65] = execute_out[54:50]; // reg_WA
	assign memory_in[70] = execute_out[88];// | */MemEn;
	assign memory_in[71] = execute_out[92];
	Register #(72) memory(memory_in, new_clock, ~StallMe, memory_out);

// --------------------------------------- //
// --------------------------------------- //
// WRITEBACK BLOCK	                      //
// --------------------------------------- //
// --------------------------------------- //	
	
// Hazard unit
// ---------------------------------------
	Hazard_unit hazard_unit(execute_out[54:50], memory_out[69:65], decode_out[137:133], decode_out[142:138], decode_out[79:75], fetch_out[9:5], fetch_out[4:0], 
		execute_out[49], memory_out[64], MemtoReg, decode_out[80], execute_out[55], type_out[1], type_out[3], type_out[2], branch_out, 
		forward_AD, forward_BD, stall_PC, stall_FD, clear_FD, clear_DE, forward_AE, forward_BE);

// Register content display to HEX
// ---------------------------------------
	HEX_driver driver1(display_data[3:0], HEX0[6:0]);
	HEX_driver driver2(display_data[7:4], HEX1[6:0]);
	HEX_driver driver3(display_data[11:8], HEX2[6:0]);
	HEX_driver driver4(display_data[15:12], HEX3[6:0]);
	HEX_driver driver5(display_data[19:16], HEX4[6:0]);
	HEX_driver driver6(display_data[23:20], HEX5[6:0]);
	HEX_driver driver7(display_data[27:24], HEX6[6:0]);
	HEX_driver driver8(display_data[31:28], HEX7[6:0]);	
endmodule 