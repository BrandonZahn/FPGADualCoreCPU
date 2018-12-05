// Shared memory controller module
// -----------------------------------------------------------
module SharedMemoryController
(
	input  logic [5:0] 	Cpu0Ad, 
	input  logic [31:0] Cpu0AdData,
	output logic [31:0] Cpu0AdCtrlWd,
	input  logic 				Cpu0AdBlockStatus,  // [shared, valid]
	output logic [5:0] 	Cpu0Ad2,
	input  logic [31:0] Cpu0Ad2Rd,
	input  logic [1:0]  Cpu0Ad2Tag,
	input  logic 				Cpu0Ad2BlockStatus,
	output logic [5:0] 	Cpu0Ad3,
	input  logic [31:0] Cpu0Ad3Rd,
	input  logic [1:0]  Cpu0Ad3Tag,
	input  logic 				Cpu0Ad3BlockStatus,
	output logic [5:0] 	Cpu0Ad4,
	input  logic [31:0] Cpu0Ad4Rd,
	input  logic [1:0]  Cpu0Ad4Tag,
	input  logic        Cpu0Ad4BlockStatus,
	input  logic 		  	Cpu0MemoryWE,
	input  logic 		  	Cpu0MemoryFunct,
	input  logic        Cpu0Hit,
	output logic [2:0]  Cpu0FetchBlock,
	output logic [2:0]  Cpu0Invalidate,
	
	input  logic [5:0] 	Cpu1Ad, 
	input  logic [31:0] Cpu1AdData,
	output logic [31:0] Cpu1AdCtrlWd,
	input  logic        Cpu1AdBlockStatus,  // [valid]
	output logic [5:0] 	Cpu1Ad2,
	input  logic [31:0] Cpu1Ad2Rd,
	input  logic [1:0]  Cpu1Ad2Tag,
	input  logic        Cpu1Ad2BlockStatus,
	output logic [5:0] 	Cpu1Ad3,
	input  logic [31:0] Cpu1Ad3Rd,
	input  logic [1:0]  Cpu1Ad3Tag,
	input  logic        Cpu1Ad3BlockStatus,
	output logic [5:0] 	Cpu1Ad4,
	input  logic [31:0] Cpu1Ad4Rd,
	input  logic [1:0]  Cpu1Ad4Tag,
	input  logic        Cpu1Ad4BlockStatus,
	input  logic 		  	Cpu1MemoryWE,
	input  logic 		  	Cpu1MemoryFunct,
	input  logic        Cpu1Hit,
	output logic [2:0]  Cpu1FetchBlock,
	output logic [2:0]  Cpu1Invalidate,
	
	input  logic [5:0] 	Cpu2Ad, 
	input  logic [31:0] Cpu2AdData,
	output logic [31:0] Cpu2AdCtrlWd,
	input  logic        Cpu2AdBlockStatus,  // [valid]
	output logic [5:0] 	Cpu2Ad2,
	input  logic [31:0] Cpu2Ad2Rd,
	input  logic [1:0]  Cpu2Ad2Tag,
	input  logic        Cpu2Ad2BlockStatus,
	output logic [5:0] 	Cpu2Ad3,
	input  logic [31:0] Cpu2Ad3Rd,
	input  logic [1:0]  Cpu2Ad3Tag,
	input  logic        Cpu2Ad3BlockStatus,
	output logic [5:0] 	Cpu2Ad4,
	input  logic [31:0] Cpu2Ad4Rd,
	input  logic [1:0]  Cpu2Ad4Tag,
	input  logic        Cpu2Ad4BlockStatus,
	input  logic 		  	Cpu2MemoryWE,
	input  logic 		  	Cpu2MemoryFunct,
	input  logic        Cpu2Hit,
	output logic [2:0]  Cpu2FetchBlock,
	output logic [2:0]  Cpu2Invalidate,
	
	input  logic [5:0] 	Cpu3Ad, 
	input  logic [31:0] Cpu3AdData,
	output logic [31:0] Cpu3AdCtrlWd,
	input  logic        Cpu3AdBlockStatus,  // [valid]
	output logic [5:0] 	Cpu3Ad2,
	input  logic [31:0] Cpu3Ad2Rd,
	input  logic [1:0]  Cpu3Ad2Tag,
	input  logic        Cpu3Ad2BlockStatus,
	output logic [5:0] 	Cpu3Ad3,
	input  logic [31:0] Cpu3Ad3Rd,
	input  logic [1:0]  Cpu3Ad3Tag,
	input  logic        Cpu3Ad3BlockStatus,
	output logic [5:0] 	Cpu3Ad4,
	input  logic [31:0] Cpu3Ad4Rd,
	input  logic [1:0]  Cpu3Ad4Tag,
	input  logic        Cpu3Ad4BlockStatus,
	input  logic 		  	Cpu3MemoryWE,
	input  logic 		  	Cpu3MemoryFunct,
	input  logic        Cpu3Hit,
	output logic [2:0]  Cpu3FetchBlock,					// Fetch from one of the other L1 caches [CPU3, CPU2, CPU1]
	output logic [2:0]  Cpu3Invalidate
);

	logic cpu0Ad2Hit, cpu0Ad3Hit, cpu0Ad4Hit, cpu1Ad2Hit, cpu1Ad3Hit, cpu1Ad4Hit,
		cpu2Ad2Hit, cpu2Ad3Hit, cpu2Ad4Hit, cpu3Ad2Hit, cpu3Ad3Hit, cpu3Ad4Hit;
	logic cpu0FetchBlock, cpu1FetchBlock, cpu2FetchBlock, cpu3FetchBlock;
	
	// Link each cache with each other
	assign Cpu0Ad2 = Cpu1Ad;
	assign Cpu0Ad3 = Cpu2Ad;
	assign Cpu0Ad4 = Cpu3Ad;
	
	assign Cpu1Ad2 = Cpu0Ad;
	assign Cpu1Ad3 = Cpu2Ad;
	assign Cpu1Ad4 = Cpu3Ad;
	
	assign Cpu2Ad2 = Cpu0Ad;
	assign Cpu2Ad3 = Cpu1Ad;
	assign Cpu2Ad4 = Cpu3Ad;

	assign Cpu3Ad2 = Cpu0Ad;
	assign Cpu3Ad3 = Cpu1Ad;
	assign Cpu3Ad4 = Cpu2Ad;
	
	// Check the other caches if they have the block in it
	assign cpu0Ad2Hit = Cpu0Ad2BlockStatus & (Cpu0Ad2Tag == Cpu0Ad2[5:4]); // valid and tag hit
	assign cpu0Ad3Hit = Cpu0Ad3BlockStatus & (Cpu0Ad3Tag == Cpu0Ad3[5:4]);
	assign cpu0Ad4Hit = Cpu0Ad4BlockStatus & (Cpu0Ad4Tag == Cpu0Ad4[5:4]);
	
	assign cpu1Ad2Hit = Cpu1Ad2BlockStatus & (Cpu1Ad2Tag == Cpu1Ad2[5:4]); // valid and tag hit
	assign cpu1Ad3Hit = Cpu1Ad3BlockStatus & (Cpu1Ad3Tag == Cpu1Ad3[5:4]);
	assign cpu1Ad4Hit = Cpu1Ad4BlockStatus & (Cpu1Ad4Tag == Cpu1Ad4[5:4]);
	
	assign cpu2Ad2Hit = Cpu2Ad2BlockStatus & (Cpu2Ad2Tag == Cpu2Ad2[5:4]); // valid and tag hit
	assign cpu2Ad3Hit = Cpu2Ad3BlockStatus & (Cpu2Ad3Tag == Cpu2Ad3[5:4]);
	assign cpu2Ad4Hit = Cpu2Ad4BlockStatus & (Cpu2Ad4Tag == Cpu2Ad4[5:4]);
	
	assign cpu3Ad2Hit = Cpu3Ad2BlockStatus & (Cpu3Ad2Tag == Cpu3Ad2[5:4]); // valid and tag hit
	assign cpu3Ad3Hit = Cpu3Ad3BlockStatus & (Cpu3Ad3Tag == Cpu3Ad3[5:4]);
	assign cpu3Ad4Hit = Cpu3Ad4BlockStatus & (Cpu3Ad4Tag == Cpu3Ad4[5:4]);
	
	// Read and miss
	// ------------------------------------------------------------------------------
	assign cpu0FetchBlock = Cpu0MemoryFunct & ~Cpu0MemoryWE & ~Cpu0Hit & (cpu1Ad2Hit | cpu2Ad2Hit | cpu3Ad2Hit);
	assign cpu1FetchBlock = Cpu1MemoryFunct & ~Cpu1MemoryWE & ~Cpu1Hit & (cpu0Ad2Hit | cpu2Ad3Hit | cpu3Ad3Hit);
	assign cpu2FetchBlock = Cpu2MemoryFunct & ~Cpu2MemoryWE & ~Cpu2Hit & (cpu0Ad3Hit | cpu1Ad3Hit | cpu3Ad4Hit);
	assign cpu3FetchBlock = Cpu3MemoryFunct & ~Cpu3MemoryWE & ~Cpu3Hit & (cpu0Ad4Hit | cpu1Ad4Hit | cpu2Ad4Hit);
	
	assign Cpu0FetchBlock[0] = cpu0FetchBlock & cpu1Ad2Hit;
	assign Cpu0FetchBlock[1] = cpu0FetchBlock & cpu2Ad2Hit;
	assign Cpu0FetchBlock[2] = cpu0FetchBlock & cpu3Ad2Hit;
	
	assign Cpu1FetchBlock[0] = cpu1FetchBlock & cpu0Ad2Hit;
	assign Cpu1FetchBlock[1] = cpu1FetchBlock & cpu2Ad3Hit;
	assign Cpu1FetchBlock[2] = cpu1FetchBlock & cpu3Ad3Hit;
	
	assign Cpu2FetchBlock[0] = cpu2FetchBlock & cpu0Ad3Hit;
	assign Cpu2FetchBlock[1] = cpu2FetchBlock & cpu1Ad3Hit;
	assign Cpu2FetchBlock[2] = cpu2FetchBlock & cpu3Ad4Hit;
	
	assign Cpu3FetchBlock[0] = cpu3FetchBlock & cpu0Ad4Hit;
	assign Cpu3FetchBlock[1] = cpu3FetchBlock & cpu1Ad4Hit;
	assign Cpu3FetchBlock[2] = cpu3FetchBlock & cpu2Ad4Hit;

	// Get the data from the other cache
	assign Cpu0AdCtrlWd = cpu1Ad2Hit ? Cpu1Ad2Rd : (cpu2Ad2Hit ? Cpu2Ad2Rd : (cpu3Ad2Hit ? Cpu3Ad2Rd : 32'dz));
	assign Cpu1AdCtrlWd = cpu0Ad2Hit ? Cpu0Ad2Rd : (cpu2Ad3Hit ? Cpu2Ad3Rd : (cpu3Ad3Hit ? Cpu3Ad3Rd : 32'dz));
	assign Cpu2AdCtrlWd = cpu0Ad3Hit ? Cpu0Ad3Rd : (cpu1Ad3Hit ? Cpu1Ad3Rd : (cpu3Ad4Hit ? Cpu3Ad4Rd : 32'dz));
	assign Cpu3AdCtrlWd = cpu0Ad4Hit ? Cpu0Ad4Rd : (cpu1Ad4Hit ? Cpu1Ad4Rd : (cpu2Ad4Hit ? Cpu2Ad4Rd : 32'dz));
		
	// Write and hit OR Write and miss (Invalidate the same blocks in other caches when a write occurs)
	// ------------------------------------------------------------------------------
	assign Cpu0Invalidate[0] = Cpu1MemoryFunct & Cpu1MemoryWE & cpu0Ad2Hit;
	assign Cpu0Invalidate[1] = Cpu2MemoryFunct & Cpu2MemoryWE & cpu0Ad3Hit;
	assign Cpu0Invalidate[2] = Cpu3MemoryFunct & Cpu3MemoryWE & cpu0Ad4Hit;
	
	assign Cpu1Invalidate[0] = Cpu0MemoryFunct & Cpu0MemoryWE & cpu1Ad2Hit;
	assign Cpu1Invalidate[1] = Cpu2MemoryFunct & Cpu2MemoryWE & cpu1Ad3Hit;
	assign Cpu1Invalidate[2] = Cpu3MemoryFunct & Cpu3MemoryWE & cpu1Ad4Hit;

	assign Cpu2Invalidate[0] = Cpu0MemoryFunct & Cpu0MemoryWE & cpu2Ad2Hit;
	assign Cpu2Invalidate[1] = Cpu1MemoryFunct & Cpu1MemoryWE & cpu2Ad3Hit;
	assign Cpu2Invalidate[2] = Cpu3MemoryFunct & Cpu3MemoryWE & cpu2Ad4Hit;

	assign Cpu3Invalidate[0] = Cpu0MemoryFunct & Cpu0MemoryWE & cpu3Ad2Hit;
	assign Cpu3Invalidate[1] = Cpu1MemoryFunct & Cpu1MemoryWE & cpu3Ad3Hit;
	assign Cpu3Invalidate[2] = Cpu2MemoryFunct & Cpu2MemoryWE & cpu3Ad4Hit;
	
endmodule 

// 4-way Bus arbiter module
// -----------------------------------------------------------
module BusArbiter
(
	input  logic 		 	 Clock,
	input  logic 	     Enable,
	input  logic 		 	 Reset,
	input  logic [3:0] Requests,
	output logic [3:0] Grants
);
	
	logic [3:0] ring, ringed;
	initial ringed = 4'b0001;
	assign ring = Reset ? 4'b0001 : ringed;
	always @ (posedge Clock) begin
		if (Enable & Reset) ringed <= 4'b0001;
		else if (Enable) ringed <= {ring[0], ring[3:1]};
	end
	
	logic [3:0] priority0In, priority1In, priority2In, priority3In, priority4In;
	logic [4:0] priority0Out, priority1Out, priority2Out, priority3Out, priority4Out; 
	
	assign priority0In = ring[0] ? {Requests[3], Requests[2], Requests[1], Requests[0]} : 4'd0;
	assign priority1In = ring[1] ? {Requests[0], Requests[3], Requests[2], Requests[1]} : 4'd0;
	assign priority2In = ring[2] ? {Requests[1], Requests[0], Requests[3], Requests[2]} : 4'd0;
	assign priority3In = ring[3] ? {Requests[2], Requests[1], Requests[0], Requests[3]} : 4'd0;

	Priority_core #(4) priority0 (priority0In,  priority0Out); 
	Priority_core #(4) priority1 (priority1In,  priority1Out); 
	Priority_core #(4) priority2 (priority2In,  priority2Out); 
	Priority_core #(4) priority3 (priority3In,  priority3Out); 

	assign Grants[0] = (priority0Out[1] | priority1Out[4] | priority2Out[3] | priority3Out[2]); 
	assign Grants[1] = (priority0Out[2] | priority1Out[1] | priority2Out[4] | priority3Out[3]); 
	assign Grants[2] = (priority0Out[3] | priority1Out[2] | priority2Out[1] | priority3Out[4]); 
	assign Grants[3] = (priority0Out[4] | priority1Out[3] | priority2Out[2] | priority3Out[1]);

endmodule 

// 8-way Bus arbiter module
// -----------------------------------------------------------
module BusArbiter8way
(
	input  logic 		 		Clock,
	input  logic 	     	Enable,
	input  logic 		 		Reset,
	input  logic [7:0] 	Requests,
	output logic [7:0]  Grants
);
	
	logic [7:0] ring, ringed;
	initial ringed = 8'b00000001;
	assign ring = Reset ? 8'b00000001 : ringed;
	always @ (posedge Clock) begin
		if (Enable & Reset) ringed <= 8'b00000001;
		else if (Enable) ringed <= {ring[0], ring[7:1]};
	end
	
	logic [7:0] priority0In, priority1In, priority2In, priority3In, priority4In, priority5In, priority6In, priority7In;
	logic [8:0] priority0Out, priority1Out, priority2Out, priority3Out, priority4Out, priority5Out, priority6Out, priority7Out; 
	
	assign priority0In = ring[0] ? {Requests[7], Requests[6], Requests[5], Requests[4], Requests[3], Requests[2], Requests[1], Requests[0]} : 8'd0;
	assign priority1In = ring[1] ? {Requests[0], Requests[7], Requests[6], Requests[5], Requests[4], Requests[3], Requests[2], Requests[1]} : 8'd0;
	assign priority2In = ring[2] ? {Requests[1], Requests[0], Requests[7], Requests[6], Requests[5], Requests[4], Requests[3], Requests[2]} : 8'd0;
	assign priority3In = ring[3] ? {Requests[2], Requests[1], Requests[0], Requests[7], Requests[6], Requests[5], Requests[4], Requests[3]} : 8'd0;
	assign priority4In = ring[4] ? {Requests[3], Requests[2], Requests[1], Requests[0], Requests[7], Requests[6], Requests[5], Requests[4]} : 8'd0;
	assign priority5In = ring[5] ? {Requests[4], Requests[3], Requests[2], Requests[1], Requests[0], Requests[7], Requests[6], Requests[5]} : 8'd0;
	assign priority6In = ring[6] ? {Requests[5], Requests[4], Requests[3], Requests[2], Requests[1], Requests[0], Requests[7], Requests[6]} : 8'd0;
	assign priority7In = ring[7] ? {Requests[6], Requests[5], Requests[4], Requests[3], Requests[2], Requests[1], Requests[0], Requests[7]} : 8'd0;

	Priority_core #(8) priority0(priority0In, priority0Out);
	Priority_core #(8) priority1(priority1In, priority1Out);
	Priority_core #(8) priority2(priority2In, priority2Out);
	Priority_core #(8) priority3(priority3In, priority3Out);
	Priority_core #(8) priority4(priority4In, priority4Out);
	Priority_core #(8) priority5(priority5In, priority5Out);
	Priority_core #(8) priority6(priority6In, priority6Out);
	Priority_core #(8) priority7(priority7In, priority7Out);
	
	assign Grants[0] = priority0Out[1] | priority1Out[8] | priority2Out[7] | priority3Out[6] | priority4Out[5] | priority5Out[4] | priority6Out[3] | priority7Out[2];
	assign Grants[1] = priority0Out[2] | priority1Out[1] | priority2Out[8] | priority3Out[7] | priority4Out[6] | priority5Out[5] | priority6Out[4] | priority7Out[3];
	assign Grants[2] = priority0Out[3] | priority1Out[2] | priority2Out[1] | priority3Out[8] | priority4Out[7] | priority5Out[6] | priority6Out[5] | priority7Out[4];
	assign Grants[3] = priority0Out[4] | priority1Out[3] | priority2Out[2] | priority3Out[1] | priority4Out[8] | priority5Out[7] | priority6Out[6] | priority7Out[5];
	assign Grants[4] = priority0Out[5] | priority1Out[4] | priority2Out[3] | priority3Out[2] | priority4Out[1] | priority5Out[8] | priority6Out[7] | priority7Out[6];
	assign Grants[5] = priority0Out[6] | priority1Out[5] | priority2Out[4] | priority3Out[3] | priority4Out[2] | priority5Out[1] | priority6Out[8] | priority7Out[7];
	assign Grants[6] = priority0Out[7] | priority1Out[6] | priority2Out[5] | priority3Out[4] | priority4Out[3] | priority5Out[2] | priority6Out[1] | priority7Out[8];
	assign Grants[7] = priority0Out[8] | priority1Out[7] | priority2Out[6] | priority3Out[5] | priority4Out[4] | priority5Out[3] | priority6Out[2] | priority7Out[1];

endmodule 

// Cache module
// -----------------------------------------------------------
module Cache 
(
	input  logic 	     	MEM_FUNCT,
	input  logic 	     	Clock,
	input  logic  	    Wen,
	input  logic [5:0] 	Ad,
	input  logic [31:0] WriteData,
	input  logic [31:0] ControlWriteData, // Write data from shared control unit
	output logic        BlockStatus,      // [valid]
	output logic [31:0] Data,
	output logic 		 		Hit,
	input  logic        StallMe,
	output logic        SRAMGW,
	output logic        SRAMOE,
	
	input  logic [5:0] 	Ad2,		   	  		// Ad from CPU
	output logic [31:0] Ad2Data,			  	// Data at Ad2 
	output logic [1:0]  Ad2Tag,			  		// Tag at Ad2
	output logic        Ad2BlockStatus,
	input  logic [5:0] 	Ad3,		   	  		// Ad from CPU
	output logic [31:0] Ad3Data,			  	// Data at Ad3 
	output logic [1:0]  Ad3Tag,			  		// Tag at Ad3
	output logic        Ad3BlockStatus,
	input  logic [5:0] 	Ad4,		   	  		// Ad from CPU
	output logic [31:0] Ad4Data,			  	// Data at Ad4 
	output logic [1:0]  Ad4Tag,			  		// Tag at Ad4
	output logic        Ad4BlockStatus,
	
	input  logic [2:0]  FetchBlock,
	input  logic [2:0]  Invalidate
);
	logic [31:0]  block[3:0];
	logic [1:0]   tag[3:0];
	logic [3:0] 	valid;
	
	integer i;
	initial begin
		for (i = 0; i < 4; i = i + 1) begin
			block[i] = 32'd0;
			tag[i] = 4'd0;
			valid[i] = 1'd0;
		end
	end
	
	logic tagHit, fetchMe;	
	assign tagHit = (tag[Ad[3:2]] == Ad[5:4]) & MEM_FUNCT;
	
	assign Ad2Tag = tag[Ad2[3:2]];
	assign Ad2Data = block[Ad2[3:2]];
	assign Ad2BlockStatus = valid[Ad2[3:2]];
	
	assign Ad3Tag = tag[Ad3[3:2]];
	assign Ad3Data = block[Ad3[3:2]];
	assign Ad3BlockStatus = valid[Ad3[3:2]];
	
	assign Ad4Tag = tag[Ad4[3:2]];
	assign Ad4Data = block[Ad4[3:2]];
	assign Ad4BlockStatus = valid[Ad4[3:2]];
	
	assign fetchMe = FetchBlock[2] | FetchBlock[1] | FetchBlock[0];
	
	// Assign hit if:
	// valid, tag is hit, and memory function
	assign Hit = valid[Ad[3:2]] & tagHit & MEM_FUNCT & ~StallMe;
	
	assign Data[31:0] = fetchMe ? ControlWriteData[31:0] : block[Ad[3:2]];
	
	assign SRAMGW = Wen & MEM_FUNCT;
	assign SRAMOE = ~Hit & ~Wen & ~fetchMe & MEM_FUNCT;
	
	assign BlockStatus = valid[Ad[3:2]];
	
	always @ (posedge Clock) begin
		// Write and hit. Update cache block with WriteData and write to RAM
		if (Hit & Wen & MEM_FUNCT & ~StallMe) begin					
			block[Ad[3:2]] <= WriteData[31:0];
			valid[Ad[3:2]] <= 1'b1;
		end
		// Read and miss. Fetch data from RAM and update cache
		else if (~Hit & ~Wen & MEM_FUNCT & ~StallMe & ~fetchMe) begin		
			block[Ad[3:2]] <= WriteData[31:0];
			tag[Ad[3:2]] <= Ad[5:4];
			valid[Ad[3:2]] <= 1'b1;
		end
		// Read and miss but fetched from elsewhere
		else if (fetchMe) begin
			block[Ad[3:2]] <= ControlWriteData[31:0];
			tag[Ad[3:2]] <= Ad[5:4];
			valid[Ad[3:2]] <= 1'b1;
		end
		// Write and miss. Fetch block from memory and update cache
		else if (~Hit & Wen & MEM_FUNCT & ~StallMe) begin 		
			block[Ad[3:2]] <= WriteData[31:0];
			tag[Ad[3:2]] <= Ad[5:4];
			valid[Ad[3:2]] <= 1'b1;
		end
		
		if (Invalidate[0]) valid[Ad2[3:2]] <= 1'b0;
		if (Invalidate[1]) valid[Ad3[3:2]] <= 1'b0;
		if (Invalidate[2]) valid[Ad4[3:2]] <= 1'b0;
		
	end
endmodule

// RAM module
// -----------------------------------------------------------
module RAM #(parameter n = 5)
(
	input logic [n-1:0] Ad,
	input logic [31:0] Din,
	input logic Clock,
	input logic Enable,
	output logic [31:0] Dout
);
	logic [7:0] array[2**n-1:0];
	

	initial begin
		$readmemb("E:/UoN/ELEC4700/QuadCoreWriteThrough/RAMInitialise.txt", array);
	end
	
	assign Dout[7:0] = array[Ad];
	assign Dout[15:8] = array[Ad + 1];
    assign Dout[23:16] = array[Ad + 2];
    assign Dout[31:24] = array[Ad + 3];
  
	always @ (posedge Clock)
		if (Enable) begin
      array[Ad] <= Din[7:0];
      array[Ad + 1] <= Din[15:8];
      array[Ad + 2] <= Din[23:16];
      array[Ad + 3] <= Din[31:24];
    end
	
endmodule

// Hazard unit module
// -----------------------------------------------------------
module Hazard_unit 
(
	input logic [4:0] Memory_WA, Writeback_WA, Execute_RA1, Execute_RA2, Execute_WA, Decode_RA1, Decode_RA2,
	input logic Memory_WE, Writeback_WE, Execute_memory_WE, Execute_WE, Memory_memory_WE, branch_type1, branch_type2, jump_type, branch_out,
	output logic forward_AD, forward_BD, stall_PC, stall_FD, clear_FD, clear_DE,
	output logic [1:0] forward_AE, forward_BE
);
	assign forward_AD = Memory_WE & (Memory_WA == Decode_RA1) & (Decode_RA1 != 0);
	assign forward_BD = Memory_WE & (Memory_WA == Decode_RA2) & (Decode_RA2 != 0);
	assign forward_AE[0] = Writeback_WE & (Writeback_WA == Execute_RA1) & (Execute_RA1 != 0);
	assign forward_AE[1] = Memory_WE & (Memory_WA == Execute_RA1) & (Execute_RA1 != 0);
	assign forward_BE[0] = Writeback_WE & (Writeback_WA == Execute_RA2) & (Execute_RA2 != 0);
	assign forward_BE[1] = Memory_WE & (Memory_WA == Execute_RA2) & (Execute_RA2 != 0);

	logic lw_stall, branch_stall;
	assign lw_stall = ((Decode_RA1 == Execute_WA) | (Decode_RA2 == Execute_WA)) & Execute_memory_WE;
	assign branch_stall = ((((Execute_WA == Decode_RA1) | (Execute_WA == Decode_RA2)) & Execute_WE) | (((Memory_WA == Decode_RA1) | (Memory_WA == Decode_RA2)) & Memory_memory_WE)) & branch_out;
	
	assign stall_PC = lw_stall | branch_stall;
	assign stall_FD = lw_stall | branch_stall;
	assign clear_FD = ~(lw_stall | branch_stall) & (jump_type | branch_out);
	assign clear_DE = lw_stall;
endmodule

// Control decoder module
// -----------------------------------------------------------
module Control_Decoder
(
	input logic [31:0] Instruction,
	output logic [10:0] type_out
);	
	logic [15:0] Priority_out;
	logic [7:0] six_priority_in;
	assign six_priority_in = {Instruction[31:26], {2{1'b0}}};
	logic [3:0] four_priority_in;
	// R-type decoder
	assign four_priority_in = {Instruction[25:23], 1'b0};
	Priority_core #(8) six_priority(six_priority_in, Priority_out[8:0]);
	Priority_core #(4) four_priority(four_priority_in, Priority_out[13:9]);

	// This is the type of command
	// eg. if (Priority_out[10]) 
	assign type_out = {Priority_out[13:11], Priority_out[9], Priority_out[8:3], Priority_out[0]};
endmodule

// Priority module
// -----------------------------------------------------------
module Priority_core #(parameter n = 8)
(
	input logic  [n-1:0] A, 
	output logic [n:0]   B
);	
	parameter m = n/2, p = m + 1;
	logic [m:0] f, e;
	
	generate 
		if (n == 1) begin
			assign B = {A,~A};
		end
		else begin 
			Priority_core #(m) lower(A[m-1:0], f);
			Priority_core #(m) higher(A[n-1:m], e);
			assign B[m:0] = f & {p{e[0]}};
			assign B[n:m+1] = e[m:1];
		end
	endgenerate
endmodule 

// Program_counter module
// -----------------------------------------------------------
module Program_counter
(
	input logic Branch, Jump,
	input logic [15:0] Branch_imm,
	input logic [15:0] Jump_imm,
	input logic Clock,
	input logic enable,
	output logic [15:0] PC, PC1
);
	assign PC1 = PC + 1;
	
	// jump_target is the ROM (PC) address that we want to jump to
	logic [15:0] jump_target;
	assign jump_target = Branch ? (Branch_imm - 16'd2 + PC1) : (Jump ? Jump_imm : PC);
	
	initial begin
	PC = {16{1'd0}};
	end
	
	always @(posedge Clock) begin
		if (enable) PC <= (Branch | Jump) ? jump_target : PC1;
	end
endmodule 

// HEX driver module
// -----------------------------------------------------------
module HEX_driver 
( 
	input logic [3:0] A,
	output logic [6:0] Y
);
	assign Y[6] = (~A[3] &~A[2] & ~A[1]) | (~A[3] & A[2] & A[1] & A[0]) | 
		(A[3] & A[2] & ~A[1] & ~A[0]);
	assign Y[5] = (~A[3] & ~A[2] & A[0]) | (~A[3] & ~A[2] & A[1]) | 
		(A[3] & A[2] & ~A[1] & A[0]);
	assign Y[4] = (~A[3] & A[0]) | (~A[3] & A[2] & ~A[1]) | (~A[2] & ~A[1] & A[0]);
	assign Y[3] = (~A[2] & ~A[1] & A[0]) | (A[2] & A[1] & A[0]) | 
		(A[3] & ~A[2] & A[1] & ~A[0]) | (~A[3] & A[2] & ~A[1] & ~A[0]);
	assign Y[2] = (A[3] & A[2] & A[1]) | (A[3] & A[2] & ~A[0]) |
		(~A[3] & ~A[2] & A[1] & ~A[0]);
	assign Y[1] = (A[2] & A[1] & ~A[0]) | (A[3] & A[2] & A[1]) |
		(A[3] & A[1] & A[0]) | (A[3] & A[2] & ~A[0]) | (~A[3] & A[2] & ~A[1] & A[0]);
	assign Y[0] = (~A[3] & ~A[2] & ~A[1] & A[0]) | (A[3] & A[2] & ~A[1] & A[0]) | 
		(A[3] & ~A[2] & A[1] & A[0]) | (~A[3] & A[2] & ~A[1] & ~A[0]);	
endmodule 

// MUX module
// -----------------------------------------------------------
module MUX #(parameter n = 3, w = 5)
(
	input logic [n-1:0]  S,
	input logic [w-1:0]  A [2**n-1:0],
	output logic [w-1:0] B
);	
	assign B = A[S];
endmodule

// Register module
// -----------------------------------------------------------
module Register #(parameter n = 8)
(
	input logic [n-1:0] D,
	input logic Clock,
	input logic Enable,
	output logic [n-1:0] Q
);
	initial begin
	Q = {n{1'd0}};
	end

	always @(posedge Clock)
		if (Enable) Q <= D;
endmodule

// CPU 0 ROM module
// -----------------------------------------------------------
module CPU0ROM #(parameter n = 5, w = 8)
(
	input logic [n-1:0] Ad,
	output logic [w-1:0] Dout
);
	logic [w-1:0] array[2**n-1:0];
	assign Dout = array[Ad];
	
	initial begin
		$readmemb("E:/UoN/ELEC4700/QuadCoreWriteThrough/instructionSetcpu0.txt", array);
	end
endmodule 

// CPU 1 ROM module
// -----------------------------------------------------------
module CPU1ROM #(parameter n = 5, w = 8)
(
	input logic [n-1:0] Ad,
	output logic [w-1:0] Dout
);
	logic [w-1:0] array[2**n-1:0];
	assign Dout = array[Ad];
	
	initial begin
		$readmemb("E:/UoN/ELEC4700/QuadCoreWriteThrough/instructionSetcpu1.txt", array);
	end
endmodule 

// CPU 2 ROM module
// -----------------------------------------------------------
module CPU2ROM #(parameter n = 5, w = 8)
(
	input logic [n-1:0] Ad,
	output logic [w-1:0] Dout
);
	logic [w-1:0] array[2**n-1:0];
	assign Dout = array[Ad];
	
	initial begin
		$readmemb("E:/UoN/ELEC4700/QuadCoreWriteThrough/instructionSetcpu2.txt", array);
	end
endmodule 

// CPU 3 ROM module
// -----------------------------------------------------------
module CPU3ROM #(parameter n = 5, w = 8)
(
	input logic [n-1:0] Ad,
	output logic [w-1:0] Dout
);
	logic [w-1:0] array[2**n-1:0];
	assign Dout = array[Ad];
	
	initial begin
		$readmemb("E:/UoN/ELEC4700/QuadCoreWriteThrough/instructionSetcpu3.txt", array);
	end
endmodule 

// Register File Module
// -----------------------------------------------------------
module Register_File #(parameter n = 32)
(
	input logic clock, Reg_WE,
	input logic [4:0] Reg_WA, Reg_RA1, Reg_RA2,
	input logic [n-1:0] Reg_WD,
	output logic [n-1:0] Reg_RD1_out, Reg_RD2_out,
	input logic [8:0] display_ad,
	output logic [31:0] display_data
);
	logic [n-1:0] reg_1_out, reg_2_out, reg_3_out, reg_4_out, reg_5_out, 
		reg_6_out, reg_7_out, reg_8_out, reg_9_out, reg_10_out, 
		reg_11_out, reg_12_out, reg_13_out, reg_14_out, reg_15_out, 
		reg_16_out, reg_17_out, reg_18_out, reg_19_out, reg_20_out, 
		reg_21_out, reg_22_out, reg_23_out, reg_24_out, reg_25_out, 
		reg_26_out, reg_27_out, reg_28_out, reg_29_out, reg_30_out, 
		reg_31_out, reg_32_out;
	
	logic reg_1_en, reg_2_en, reg_3_en, reg_4_en, reg_5_en, 
		reg_6_en, reg_7_en, reg_8_en, reg_9_en, reg_10_en, 
		reg_11_en, reg_12_en, reg_13_en, reg_14_en, reg_15_en, 
		reg_16_en, reg_17_en, reg_18_en, reg_19_en, reg_20_en, 
		reg_21_en, reg_22_en, reg_23_en, reg_24_en, reg_25_en, 
		reg_26_en, reg_27_en, reg_28_en, reg_29_en, reg_30_en, 
		reg_31_en, reg_32_en;
	
	logic [31:0] dec_out;
	

	Decoder #(5) dec(Reg_WA, dec_out);
	
	assign reg_1_en = dec_out[0] & Reg_WE;
	assign reg_2_en = dec_out[1] & Reg_WE;
	assign reg_3_en = dec_out[2] & Reg_WE;
	assign reg_4_en = dec_out[3] & Reg_WE;
	assign reg_5_en = dec_out[4] & Reg_WE;
	assign reg_6_en = dec_out[5] & Reg_WE;
	assign reg_7_en = dec_out[6] & Reg_WE;
	assign reg_8_en = dec_out[7] & Reg_WE;
	assign reg_9_en = dec_out[8] & Reg_WE;
	assign reg_10_en = dec_out[9] & Reg_WE;
	assign reg_11_en = dec_out[10] & Reg_WE;
	assign reg_12_en = dec_out[11] & Reg_WE;
	assign reg_13_en = dec_out[12] & Reg_WE;
	assign reg_14_en = dec_out[13] & Reg_WE;
	assign reg_15_en = dec_out[14] & Reg_WE;
	assign reg_16_en = dec_out[15] & Reg_WE;
	assign reg_17_en = dec_out[16] & Reg_WE;
	assign reg_18_en = dec_out[17] & Reg_WE;
	assign reg_19_en = dec_out[18] & Reg_WE;
	assign reg_20_en = dec_out[19] & Reg_WE;
	assign reg_21_en = dec_out[20] & Reg_WE;
	assign reg_22_en = dec_out[21] & Reg_WE;
	assign reg_23_en = dec_out[22] & Reg_WE;
	assign reg_24_en = dec_out[23] & Reg_WE;
	assign reg_25_en = dec_out[24] & Reg_WE;
	assign reg_26_en = dec_out[25] & Reg_WE;
	assign reg_27_en = dec_out[26] & Reg_WE;
	assign reg_28_en = dec_out[27] & Reg_WE;
	assign reg_29_en = dec_out[28] & Reg_WE;
	assign reg_30_en = dec_out[29] & Reg_WE;
	assign reg_31_en = dec_out[30] & Reg_WE;
	assign reg_32_en = dec_out[31] & Reg_WE;
	
	assign reg_1_out = 32'd0;
	Register #(n) reg_2(Reg_WD, clock, reg_2_en, reg_2_out);
	Register #(n) reg_3(Reg_WD, clock, reg_3_en, reg_3_out);
	Register #(n) reg_4(Reg_WD, clock, reg_4_en, reg_4_out);
	Register #(n) reg_5(Reg_WD, clock, reg_5_en, reg_5_out);
	Register #(n) reg_6(Reg_WD, clock, reg_6_en, reg_6_out);
	Register #(n) reg_7(Reg_WD, clock, reg_7_en, reg_7_out);
	Register #(n) reg_8(Reg_WD, clock, reg_8_en, reg_8_out);
	Register #(n) reg_9(Reg_WD, clock, reg_9_en, reg_9_out);
	Register #(n) reg_10(Reg_WD, clock, reg_10_en, reg_10_out);
	Register #(n) reg_11(Reg_WD, clock, reg_11_en, reg_11_out);
	Register #(n) reg_12(Reg_WD, clock, reg_12_en, reg_12_out);
	Register #(n) reg_13(Reg_WD, clock, reg_13_en, reg_13_out);
	Register #(n) reg_14(Reg_WD, clock, reg_14_en, reg_14_out);
	Register #(n) reg_15(Reg_WD, clock, reg_15_en, reg_15_out);
	Register #(n) reg_16(Reg_WD, clock, reg_16_en, reg_16_out);
	Register #(n) reg_17(Reg_WD, clock, reg_17_en, reg_17_out);
	Register #(n) reg_18(Reg_WD, clock, reg_18_en, reg_18_out);
	Register #(n) reg_19(Reg_WD, clock, reg_19_en, reg_19_out);
	Register #(n) reg_20(Reg_WD, clock, reg_20_en, reg_20_out);
	Register #(n) reg_21(Reg_WD, clock, reg_21_en, reg_21_out);
	Register #(n) reg_22(Reg_WD, clock, reg_22_en, reg_22_out);
	Register #(n) reg_23(Reg_WD, clock, reg_23_en, reg_23_out);
	Register #(n) reg_24(Reg_WD, clock, reg_24_en, reg_24_out);
	Register #(n) reg_25(Reg_WD, clock, reg_25_en, reg_25_out);
	Register #(n) reg_26(Reg_WD, clock, reg_26_en, reg_26_out);
	Register #(n) reg_27(Reg_WD, clock, reg_27_en, reg_27_out);
	Register #(n) reg_28(Reg_WD, clock, reg_28_en, reg_28_out);
	Register #(n) reg_29(Reg_WD, clock, reg_29_en, reg_29_out);
	Register #(n) reg_30(Reg_WD, clock, reg_30_en, reg_30_out);
	Register #(n) reg_31(Reg_WD, clock, reg_31_en, reg_31_out);
	Register #(n) reg_32(Reg_WD, clock, reg_32_en, reg_32_out);
	
	logic [n-1:0] reg_mux_01_bus[31:0];
	assign reg_mux_01_bus[0] = reg_1_out;
	assign reg_mux_01_bus[1] = reg_2_out;
	assign reg_mux_01_bus[2] = reg_3_out;
	assign reg_mux_01_bus[3] = reg_4_out;
	assign reg_mux_01_bus[4] = reg_5_out;
	assign reg_mux_01_bus[5] = reg_6_out;
	assign reg_mux_01_bus[6] = reg_7_out;
	assign reg_mux_01_bus[7] = reg_8_out;
	assign reg_mux_01_bus[8] = reg_9_out;
	assign reg_mux_01_bus[9] = reg_10_out;
	assign reg_mux_01_bus[10] = reg_11_out;
	assign reg_mux_01_bus[11] = reg_12_out;
	assign reg_mux_01_bus[12] = reg_13_out;
	assign reg_mux_01_bus[13] = reg_14_out;
	assign reg_mux_01_bus[14] = reg_15_out;
	assign reg_mux_01_bus[15] = reg_16_out;
	assign reg_mux_01_bus[16] = reg_17_out;
	assign reg_mux_01_bus[17] = reg_18_out;
	assign reg_mux_01_bus[18] = reg_19_out;
	assign reg_mux_01_bus[19] = reg_20_out;
	assign reg_mux_01_bus[20] = reg_21_out;
	assign reg_mux_01_bus[21] = reg_22_out;
	assign reg_mux_01_bus[22] = reg_23_out;
	assign reg_mux_01_bus[23] = reg_24_out;
	assign reg_mux_01_bus[24] = reg_25_out;
	assign reg_mux_01_bus[25] = reg_26_out;
	assign reg_mux_01_bus[26] = reg_27_out;
	assign reg_mux_01_bus[27] = reg_28_out;
	assign reg_mux_01_bus[28] = reg_29_out;
	assign reg_mux_01_bus[29] = reg_30_out;
	assign reg_mux_01_bus[30] = reg_31_out;
	assign reg_mux_01_bus[31] = reg_32_out;
	
	MUX #(5,32) reg_mux_01(Reg_RA1, reg_mux_01_bus, Reg_RD1_out);
	
	logic [n-1:0] reg_mux_02_bus[31:0];
	assign reg_mux_02_bus[0] = reg_1_out;
	assign reg_mux_02_bus[1] = reg_2_out;
	assign reg_mux_02_bus[2] = reg_3_out;
	assign reg_mux_02_bus[3] = reg_4_out;
	assign reg_mux_02_bus[4] = reg_5_out;
	assign reg_mux_02_bus[5] = reg_6_out;
	assign reg_mux_02_bus[6] = reg_7_out;
	assign reg_mux_02_bus[7] = reg_8_out;
	assign reg_mux_02_bus[8] = reg_9_out;
	assign reg_mux_02_bus[9] = reg_10_out;
	assign reg_mux_02_bus[10] = reg_11_out;
	assign reg_mux_02_bus[11] = reg_12_out;
	assign reg_mux_02_bus[12] = reg_13_out;
	assign reg_mux_02_bus[13] = reg_14_out;
	assign reg_mux_02_bus[14] = reg_15_out;
	assign reg_mux_02_bus[15] = reg_16_out;
	assign reg_mux_02_bus[16] = reg_17_out;
	assign reg_mux_02_bus[17] = reg_18_out;
	assign reg_mux_02_bus[18] = reg_19_out;
	assign reg_mux_02_bus[19] = reg_20_out;
	assign reg_mux_02_bus[20] = reg_21_out;
	assign reg_mux_02_bus[21] = reg_22_out;
	assign reg_mux_02_bus[22] = reg_23_out;
	assign reg_mux_02_bus[23] = reg_24_out;
	assign reg_mux_02_bus[24] = reg_25_out;
	assign reg_mux_02_bus[25] = reg_26_out;
	assign reg_mux_02_bus[26] = reg_27_out;
	assign reg_mux_02_bus[27] = reg_28_out;
	assign reg_mux_02_bus[28] = reg_29_out;
	assign reg_mux_02_bus[29] = reg_30_out;
	assign reg_mux_02_bus[30] = reg_31_out;
	assign reg_mux_02_bus[31] = reg_32_out;
	
	MUX #(5,32) reg_mux_02(Reg_RA2, reg_mux_02_bus, Reg_RD2_out);	
	
	assign display_data = display_ad[0] ? reg_1_out : 
		(display_ad[1] ? reg_2_out : (display_ad[2] ? reg_3_out : (display_ad[3] ? reg_4_out : 
		(display_ad[4] ? reg_5_out : (display_ad[5] ? reg_6_out : (display_ad[6] ? reg_7_out : reg_8_out))))));
		
endmodule 

//Decoder
// -----------------------------------------------------------
module Decoder #(parameter n = 3)
(
	input logic [n-1:0] a,
	output logic [2**n-1:0] y
);

	parameter w = 2**n-1;
	
	always_comb begin
		y = {w{1'b0}};
		y[a] = 1'b1;
	end
	
endmodule

// Memory interface module
// -----------------------------------------------------------
module Memory_interface #(parameter n = 32)
(
	input  logic  		   MEM_FUNCT,
	input  logic [n-1:0] WD,
	input  logic [2:0]   Op,
	input  logic [5:0]   Ad, // this size affects performance
	input  logic 		     Clock,
	output logic [n-1:0] RD,
	output logic  		   MemEn,
	input  logic [31:0]  SRAMRD,
	output logic [31:0]  SRAMWD,
	output logic         SRAMGW,
	output logic 		     SRAMOE,
	input  logic  		   StallMe,
	output logic [31:0]  CachedData,
	input  logic [31:0]  ControlWriteData, // Write data from shared control unit
	output logic         BlockStatus,      // [shared, valid]
	input  logic [5:0]   Ad2,		   	  // Ad from other CPU
	output logic [31:0]  Ad2Data,			  // Data at Ad2 
	output logic [1:0]   Ad2Tag,			  // Tag at Ad2
	output logic         Ad2BlockStatus,
	input  logic [5:0] 	 Ad3,		   	  		// Ad from CPU
	output logic [31:0]  Ad3Data,			  	// Data at Ad3 
	output logic [1:0]   Ad3Tag,			  		// Tag at Ad3
	output logic         Ad3BlockStatus,
	input  logic [5:0] 	 Ad4,		   	  		// Ad from CPU
	output logic [31:0]  Ad4Data,			  	// Data at Ad4 
	output logic [1:0]   Ad4Tag,			  		// Tag at Ad4
	output logic         Ad4BlockStatus,
	output logic         Hit,
	input  logic [2:0]   FetchBlock,
	input  logic [2:0]   Invalidate
);
	logic [n-1:0] Mout, MemIn, cachedData, cacheWD;
	logic fetchMe;
	assign fetchMe = (FetchBlock[2] | FetchBlock[1] | FetchBlock[0]);
	
	//logic [1:0] Stall_counter, Stall_counter1;
	
	assign SRAMWD = SRAMGW ? MemIn : 32'dz;
	assign cacheWD = SRAMOE ? Mout : MemIn;
	//RAM #(6) Mem_inter_RAM({Ad[5:2], 2'b00}, MemIn, Clock, RAMEN, RAMout);

// L1 cache
// ---------------------------------------
	Cache L1_cache(MEM_FUNCT, Clock, MemEn, Ad, cacheWD, ControlWriteData,
		BlockStatus, cachedData, Hit, StallMe, SRAMGW, SRAMOE, Ad2, Ad2Data, 
		Ad2Tag, Ad2BlockStatus, Ad3, Ad3Data, Ad3Tag, Ad3BlockStatus, Ad4, Ad4Data, 
		Ad4Tag, Ad4BlockStatus, FetchBlock, Invalidate);
	
	assign Mout = (Hit | fetchMe) ? cachedData : SRAMRD;
	assign CachedData = cachedData;
	
	/*assign SRAMSTALL = (SRAMOE | ((~Stall_counter[1] & Stall_counter[0]) | 
		(Stall_counter[1] & ~Stall_counter[0]))) ? 1'b1: 1'b0;
	
	initial Stall_counter = 2'd0;

	assign Stall_counter1 = Stall_counter + 1;
	
	always @(posedge Clock)
		if (SRAMSTALL) Stall_counter <= Stall_counter1;
		else if (WriteBack) Stall_counter <= 2'b01;
		else if (~SRAMSTALL) Stall_counter <= 2'd0;*/
			
	logic [n-1:0] Mem_Mux_01_Out, Mem_Mux_02_Out, Mem_Mux_03_Out, Mem_Mux_04_Out, 
		Mem_Mux_05_Out, Mem_Mux_06_Out, Mem_Mux_08_Out, Mem_Mux_09_Out, Mem_Mux_10_Out, 
		Mem_Mux_11_Out, Mem_Mux_12_Out, Mem_Mux_13_Out, Mem_Mux_14_Out, Mem_Mux_15_Out, 
		Mem_Mux_16_Out, Mem_Mux_17_Out, Mem_Mux_18_Out, Mem_Mux_19_Out, Mem_Mux_20_Out;
	
	assign Mem_Mux_01_Out = Ad[0] ? {Mout[n-1:16] , WD[7:0] , Mout[7:0]} : {Mout[n-1:8] , WD[7:0]};
	assign Mem_Mux_02_Out = Ad[0] ? {WD[7:0] , Mout[23:0]} : {Mout[n-1:24] , WD[7:0] , Mout[15:0]};
	assign Mem_Mux_03_Out = Ad[0] ? {Mout[n-1:24] , WD[15:0] , Mout[7:0]} : {Mout[n-1:16] , WD[15:0]};
	assign Mem_Mux_04_Out = Ad[1] ? Mem_Mux_02_Out : Mem_Mux_01_Out;
	assign Mem_Mux_05_Out = Ad[1] ? {WD[15:0] , Mout[15:0]} : Mem_Mux_03_Out;
	assign Mem_Mux_06_Out = Op[1] ? Mem_Mux_05_Out : Mem_Mux_04_Out;
	assign MemEn = (Op[0] & ~Op[1] & Op[2]) | (Op[1] & Op[2]);
	assign MemIn = (Op[0] & Op[1] & Op[2]) ? WD : Mem_Mux_06_Out;
	assign Mem_Mux_08_Out = Op[1] ? {{24{1'b0}} , Mout[7:0]} : {{24{Mout[7]}} , Mout[7:0]};
	assign Mem_Mux_09_Out = Op[1] ? {{24{1'b0}} , Mout[15:8]} : {{24{Mout[15]}} , Mout[15:8]};
	assign Mem_Mux_10_Out = Op[1] ? {{24{1'b0}} , Mout[23:16]} : {{24{Mout[23]}} , Mout[23:16]};
	assign Mem_Mux_11_Out = Op[1] ? {{24{1'b0}} , Mout[31:24]} : {{24{Mout[31]}} , Mout[31:24]};
	assign Mem_Mux_12_Out = Op[1] ? {{16{1'b0}} , Mout[15:0]} : {{16{Mout[15]}} , Mout[15:0]};
	assign Mem_Mux_13_Out = Op[1] ? {{16{1'b0}} , Mout[23:8]} : {{16{Mout[23]}} , Mout[23:8]};
	assign Mem_Mux_14_Out = Op[1] ? {{16{1'b0}}, Mout[31:16]} : {{16{Mout[31]}} , Mout[31:16]};
	assign Mem_Mux_15_Out = Ad[0] ? Mem_Mux_09_Out : Mem_Mux_08_Out;
	assign Mem_Mux_16_Out = Ad[0] ? Mem_Mux_11_Out : Mem_Mux_10_Out;
	assign Mem_Mux_17_Out = Ad[0] ? Mem_Mux_13_Out : Mem_Mux_12_Out;
	assign Mem_Mux_18_Out = Ad[1] ? Mem_Mux_16_Out : Mem_Mux_15_Out;
	assign Mem_Mux_19_Out = Ad[1] ? Mem_Mux_14_Out : Mem_Mux_17_Out;
	assign Mem_Mux_20_Out = Op[0] ? Mem_Mux_19_Out : Mem_Mux_18_Out;
	
	assign RD = Op[2] ? Mout : Mem_Mux_20_Out;
endmodule 

// Shifter module
// -----------------------------------------------------------
module Shifter
(
	input logic  [31:0] A, 
	input logic  [4:0] Shamt,
	input logic  [31:0] B,
	input logic  [2:0] Funct, 
	output logic [31:0] Y
);
	logic [31:0] MUX_out;
	logic [4:0] shift_amount;
	assign shift_amount = Funct[2] ? B[4:0] : Shamt;
	assign MUX_out = (Funct[1] ? $signed(A) >>> shift_amount : (Funct[0] ? A >> shift_amount : A << shift_amount));
	
	assign Y = MUX_out;
endmodule

// MUL/DIV module
// -----------------------------------------------------------
module MUL_DIV #(parameter n = 8)
(
	input logic [n-1:0] A, B,
	input logic [3:0] Funct,
	input logic Clock,
	output logic [n-1:0] Y
);
	logic [n-1:0] b_line;
	logic [n-1:0] c_line;
	logic [n-1:0] hi_line;
	logic [n-1:0] lo_line;
	logic [n-1:0] remain_line;
	logic [n-1:0] quotient_line;
	
	assign b_line = A;
	assign c_line = B;
	
	assign {hi_line,lo_line} = b_line * c_line;
	
	assign quotient_line = b_line / c_line;
	assign remain_line = b_line % c_line;
	
	logic [n-1:0] mux_h_r_out;
	assign mux_h_r_out = Funct[1] ? remain_line : hi_line;

	logic [n-1:0] mux_l_q_out;
	assign mux_l_q_out = Funct[1] ? quotient_line : lo_line;

	logic [n-1:0] mux_a_h_r_out;
	assign mux_a_h_r_out = Funct[3] ? mux_h_r_out : b_line;
	
	logic [n-1:0] mux_a_l_q_out;
	assign mux_a_l_q_out = Funct[3] ? mux_l_q_out : b_line;
	
	logic enable_hi, enable_lo;
	assign enable_hi = Funct[3] | (~Funct[3] & ~Funct[2] & ~Funct[1] & Funct[0]);
	assign enable_lo = Funct[3] | (~Funct[3] & ~Funct[2] & Funct[1] & Funct[0]);

	logic [n-1:0] reg_hi_out;
	Register #(n) register_hi(mux_a_h_r_out, Clock, enable_hi, reg_hi_out);
	
	logic [n-1:0] reg_lo_out;
	Register #(n) register_lo(mux_a_l_q_out, Clock, enable_lo, reg_lo_out);

	assign Y = Funct[1] ? reg_lo_out : reg_hi_out;	
endmodule

//ALU module
module ALU #(parameter n = 8)
(
	input logic [n-1:0] A,
	input logic [n-1:0] B,
	input logic [3:0] Funct,
	output logic [n-1:0] Y,
	output logic Ov, Cout
);
	logic [n-1:0] B_notted;
	assign B_notted = ~B;
	
	logic [n-1:0] mux_one_out;
	assign mux_one_out = Funct[1] ? B_notted : B;
	
	logic [n-1:0] add_out;
	assign {Cout, add_out} = A + mux_one_out + Funct[1];
	
	logic A_msb, B_msb, xnor_gate_out;
	assign A_msb = A[n-1];
	assign B_msb = B[n-1];
	assign xnor_gate_out = ~(A_msb ^ B_msb);
	
	logic add_out_msb, xor_gate_one_out;
	assign add_out_msb = add_out[n-1];
	assign xor_gate_one_out = A_msb ^ add_out_msb;
	
	logic and_gate_one_out;
	assign and_gate_one_out = xnor_gate_out & xor_gate_one_out;
	
	logic xor_gate_two_out;
	assign xor_gate_two_out = Cout ^ Funct[1];
	
	logic mux_two_out;
	assign mux_two_out = Funct[0] ? xor_gate_two_out : and_gate_one_out;
	assign Ov = mux_two_out;
	
	logic mux_three_out;
	assign mux_three_out = and_gate_one_out ? Cout : add_out_msb;
	
	logic Cout_notted;
	assign Cout_notted = ~Cout;
	
	logic mux_four_out;
	assign mux_four_out = Funct[0] ? Cout_notted : mux_three_out;
	
	logic [n-1:0] mux_five_out;
	assign mux_five_out = Funct[3] ? {{n-1{1'b0}}, mux_four_out} : add_out;
	
	logic [n-1:0] and_logic_out, or_logic_out, xor_logic_out, nor_logic_out;
	assign and_logic_out = A & B;
	assign or_logic_out = A | B;
	assign xor_logic_out = A ^ B;
	assign nor_logic_out = ~(A | B);
	
	logic [n-1:0] mux_six_in [4];
	logic [n-1:0] mux_six_out;
	assign mux_six_in[0] = nor_logic_out;
	assign mux_six_in[1] = xor_logic_out;
	assign mux_six_in[2] = or_logic_out;
	assign mux_six_in[3] = and_logic_out;
	MUX #(2, n) mux_six(Funct[1:0], mux_six_in, mux_six_out);
	
	assign Y = Funct[2] ? mux_six_out : mux_five_out;
endmodule 

// Immediate Module
// -----------------------------------------------------------
module Immediate #(parameter n = 32)
(
	input logic [13:0] Imm_input,
	input logic ImmEn,
	input logic [2:0] ImmF,
	input logic [1:0] Up_Imm_Value,
	output logic [n-1:0] Imm_out
);
	logic [n-1:0] SignImm, ZeroImm, CompImm;
	logic [15:0] imm_input_bus;
	assign imm_input_bus = ImmEn ? {Up_Imm_Value, Imm_input} : {Up_Imm_Value, Imm_input};
	assign SignImm = ImmEn ? {{16{imm_input_bus[15]}}, imm_input_bus[15:0]} : {{16{imm_input_bus[15]}} , imm_input_bus[15:0]};
	assign ZeroImm = ImmEn ? {{16{1'b0}}, imm_input_bus[15:0]} : {{16{1'b0}} , imm_input_bus[15:0]};
	assign CompImm = {~(imm_input_bus[15:0]), {16{1'b1}}};
	logic [n-1:0] Imm_outPost;
	assign Imm_outPost = ImmF[0] ? ZeroImm : SignImm;
	assign Imm_out = (ImmEn & ImmF[2]) ? ((ImmF[0] & ImmF[1]) ? CompImm : ZeroImm) : Imm_outPost;
endmodule 

// Branch Module
// -----------------------------------------------------------
module Branch #(parameter n = 32)
(
	input logic B1, B2,
	input logic [1:0] Branch_Funct,
	input logic [n-1:0] Branch_rt, Branch_rs,
	output logic Branch_out
);
	logic [n-1:0] Branch_rt_sel;
	assign  Branch_rt_sel = Branch_rt ^ Branch_rs;
	
	logic branch_mux_in_1, branch_mux_in_2, branch_mux_out;
	assign branch_mux_in_1 = (Branch_Funct[0] ^ (~(Branch_rt_sel[0]|Branch_rt_sel[1]|Branch_rt_sel[2]|Branch_rt_sel[3]
	|Branch_rt_sel[4]|Branch_rt_sel[5]|Branch_rt_sel[6]|Branch_rt_sel[7]|Branch_rt_sel[8]|Branch_rt_sel[9]|Branch_rt_sel[10]
	|Branch_rt_sel[11]|Branch_rt_sel[12]|Branch_rt_sel[13]|Branch_rt_sel[14]|Branch_rt_sel[15]|Branch_rt_sel[16]|Branch_rt_sel[17]
	|Branch_rt_sel[18]|Branch_rt_sel[19]|Branch_rt_sel[20]|Branch_rt_sel[21]|Branch_rt_sel[22]|Branch_rt_sel[23]|Branch_rt_sel[24]
	|Branch_rt_sel[25]|Branch_rt_sel[26]|Branch_rt_sel[27]|Branch_rt_sel[28]|Branch_rt_sel[29]|Branch_rt_sel[30]|Branch_rt_sel[31])));
	assign branch_mux_in_2 = (Branch_Funct[0] ^ (Branch_rs[31]|~(Branch_rs[0]|Branch_rs[1]|Branch_rs[2]|Branch_rs[3]|Branch_rs[4]
	|Branch_rs[5]|Branch_rs[6]|Branch_rs[7]|Branch_rs[8]|Branch_rs[9]|Branch_rs[10]|Branch_rs[11]|Branch_rs[12]|Branch_rs[13]|Branch_rs[14]
	|Branch_rs[15]|Branch_rs[16]|Branch_rs[17]|Branch_rs[18]|Branch_rs[19]|Branch_rs[20]|Branch_rs[21]|Branch_rs[22]|Branch_rs[23]|Branch_rs[24]
	|Branch_rs[25]|Branch_rs[26]|Branch_rs[27]|Branch_rs[28]|Branch_rs[29]|Branch_rs[30]|Branch_rs[31])));
	assign branch_mux_out = Branch_Funct[1] ?  branch_mux_in_2 : branch_mux_in_1;
	assign Branch_out=(B2 & branch_mux_out) | (B1 & (Branch_rs[31] ^ Branch_rt_sel[0]));
endmodule 

//module SSRAM
//(
//	output logic [18:0] address,  // Address inputs
//	inout  		 [31:0] data, 		// Data inputs/outputs
//	output logic chip_EN,  	   	// Chip enable
//	output logic output_EN,     	// Output enable
//	output logic write_EN,      	// Write enable
//);
//
//  logic enable_io_output; // Send data to SRAM?
//  logic [15:0] io_output;
//  logic enable_ledr_output; // Display data on LEDR?
//  logic [17:0] ledr_output;
//
//  // Remember: KEYs are HIGH (1) when NOT pressed,
//  // so it really should be called KEY_n
//
//  // We always leave the ASRAM chip enabled
//  assign SRAM_CE_N = 1'b0;
//  
//  // We always do word-at-a-time access
//  assign SRAM_LB_N = 1'b0;
//  assign SRAM_UB_N = 1'b0;
//
//  // Switches 0-9  : Data input        (10)
//  // Switches 12-17: Address selection (6)
//  // Key 0: Output enable
//  // Key 1: Write enable
//  
//  // Keys
//  assign SRAM_OE_N = KEY[0];
//  assign SRAM_WE_N = KEY[1];
//  
//  // Address
//  assign SRAM_ADDR = {14'b0, SW[17:12]};
//
//  // Data - this is the hard part.
//  // We load the LEDR from the SRAM_DQ when we're reading,
//  // but we load the SRAM_DQ from the switches when we're writing.
//  // We don't output to LEDR unless we're reading.
//  assign SRAM_DQ = enable_io_output ? io_output : 16'bZ;
//  assign LEDR = enable_ledr_output ? ledr_output : 18'bZ;
//  
//  always_comb begin
// io_output = 16'b0;       // Default output value (to avoid a latch)
// enable_io_output = 1'b0; // Don't enable output if unnecessary
// ledr_output = 18'bZ;     // Wonder if this will work...
// enable_ledr_output = 1'b0;
// if (~KEY[1]) begin // We're writing, so use I/O lines as output
//io_output = {6'b0, SW[9:0]};
//enable_io_output = 1'b1;
// end else if (~KEY[0]) begin // We're reading and NOT writing, so use I/O lines as input
//ledr_output = {2'b0, SRAM_DQ};
//enable_ledr_output = 1'b1;
// end
//  end // always_comb
//
//endmodule