module ELEC4700
(
	input  logic 				new_clock,//CLOCK_50,
	input  logic [17:0] SW,
	input  logic  [3:0] KEY, 
	output logic [17:0] LEDR,
	output logic  [8:0] LEDG,
	output logic  [7:0] HEX0,
	output logic  [7:0] HEX1,
	output logic  [7:0] HEX2,
	output logic  [7:0] HEX3,
	output logic  [7:0] HEX4,
	output logic  [7:0] HEX5,
	output logic  [7:0] HEX6,
	output logic  [7:0] HEX7,
	output logic 		  	SRAM_CLK, // SSRAM clock
	output logic 		  	SRAM_CE1_N, SRAM_CE2, SRAM_CE3_N, // SSRAM enables 
	output logic [18:0] SRAM_A, // SSRAM R/W address
	//inout 		   [31:0] SRAM_DQ, // R/W data
	output logic 		  	SRAM_ADSC_N, SRAM_ADSP_N, SRAM_ADV_N, // burst stuff
	output logic 		  	SRAM_BE_N0, SRAM_BE_N1, SRAM_BE_N2, SRAM_BE_N3, // byte W enables
	output logic 		  	SRAM_DPA0, SRAM_DPA1, SRAM_DPA2, SRAM_DPA3, // byte parity
	output logic 		  	SRAM_GW_N, SRAM_OE_N, SRAM_WE_N // Write enables
); 

// Set up the clock for the CPU
// ---------------------------------------
	//logic [31:0] all_clocks;
	//always @(posedge CLOCK_50) begin
	//	all_clocks <= all_clocks + 1;
	//end
	//logic new_clock;
	//assign new_clock = all_clocks[25]; // We can change the clock speed with this bit
	//assign LEDG[8] = new_clock;

// Local logic for CPUs
// ---------------------------------------	
	logic [7:0] cpu0hex0, cpu0hex1, cpu0hex2, cpu0hex3, 
		cpu0hex4, cpu0hex5, cpu0hex6, cpu0hex7;
	logic [17:0] cpu0ledr;
	logic [7:0] cpu0ledg;
	logic [5:0] cpu0sramad;
	logic [31:0] cpu0sramrd, cpu0sramwd, cpu0cacherd;//, cpu0cachewd;
	logic cpu0sramgw, cpu0sramoe, cpu0stallMe;//, cpu0ramStall;
	logic cpu0memorywe, cpu0memoryfunct, cpu0hit;
	logic [2:0] cpu0fetchblock, cpu0invalidate;
	logic [31:0] cpu0controlwd;
	logic cpu0blockstatus;
	logic [5:0] cpu0ad2, cpu0ad3, cpu0ad4;
	logic [31:0] cpu0ad2rd, cpu0ad3rd, cpu0ad4rd;
	logic [1:0]  cpu0ad2tag, cpu0ad3tag, cpu0ad4tag;
	logic cpu0ad2blockstatus, cpu0ad3blockstatus, cpu0ad4blockstatus;

	logic [7:0] cpu1hex0, cpu1hex1, cpu1hex2, cpu1hex3, 
		cpu1hex4, cpu1hex5, cpu1hex6, cpu1hex7;
	logic [17:0] cpu1ledr;
	logic [7:0] cpu1ledg;
	logic [5:0] cpu1sramad;
	logic [31:0] cpu1sramrd, cpu1sramwd, cpu1cacherd;//, cpu1cachewd;
	logic cpu1sramgw, cpu1sramoe, cpu1stallMe;//, cpu1ramStall;
	logic cpu1memorywe, cpu1memoryfunct, cpu1hit;
	logic [2:0] cpu1fetchblock, cpu1invalidate;
	logic [31:0] cpu1controlwd;
	logic cpu1blockstatus;
	logic [5:0] cpu1ad2, cpu1ad3, cpu1ad4;
	logic [31:0] cpu1ad2rd, cpu1ad3rd, cpu1ad4rd;
	logic [1:0]  cpu1ad2tag, cpu1ad3tag, cpu1ad4tag;
	logic cpu1ad2blockstatus, cpu1ad3blockstatus, cpu1ad4blockstatus;
	
	logic [7:0] cpu2hex0, cpu2hex1, cpu2hex2, cpu2hex3, 
		cpu2hex4, cpu2hex5, cpu2hex6, cpu2hex7;
	logic [17:0] cpu2ledr;
	logic [7:0] cpu2ledg;
	logic [5:0] cpu2sramad;
	logic [31:0] cpu2sramrd, cpu2sramwd, cpu2cacherd;//, cpu2cachewd;
	logic cpu2sramgw, cpu2sramoe, cpu2stallMe;//, cpu2ramStall;
	logic cpu2memorywe, cpu2memoryfunct, cpu2hit;
	logic [2:0] cpu2fetchblock,	cpu2invalidate;
	logic [31:0] cpu2controlwd;
	logic cpu2blockstatus;
	logic [5:0] cpu2ad2, cpu2ad3, cpu2ad4;
	logic [31:0] cpu2ad2rd, cpu2ad3rd, cpu2ad4rd;
	logic [1:0]  cpu2ad2tag, cpu2ad3tag, cpu2ad4tag;
	logic cpu2ad2blockstatus, cpu2ad3blockstatus, cpu2ad4blockstatus;
	
	logic [7:0] cpu3hex0, cpu3hex1, cpu3hex2, cpu3hex3, 
		cpu3hex4, cpu3hex5, cpu3hex6, cpu3hex7;
	logic [17:0] cpu3ledr;
	logic [7:0] cpu3ledg;
	logic [5:0] cpu3sramad;
	logic [31:0] cpu3sramrd, cpu3sramwd, cpu3cacherd;//, cpu3cachewd;
	logic cpu3sramgw, cpu3sramoe, cpu3stallMe;//, cpu3ramStall;
	logic cpu3memorywe, cpu3memoryfunct, cpu3hit;
	logic [2:0] cpu3fetchblock,	cpu3invalidate;
	logic [31:0] cpu3controlwd;
	logic cpu3blockstatus;
	logic [5:0] cpu3ad2, cpu3ad3, cpu3ad4;
	logic [31:0] cpu3ad2rd, cpu3ad3rd, cpu3ad4rd;
	logic [1:0]  cpu3ad2tag, cpu3ad3tag, cpu3ad4tag;
	logic cpu3ad2blockstatus, cpu3ad3blockstatus, cpu3ad4blockstatus;
	
// Set up the SSRAM
// ---------------------------------------	
	assign SRAM_CLK = new_clock;//CLOCK_50;
	
	assign SRAM_CE1_N = 1'b0; 
	assign SRAM_CE2 = 1'b1;
	assign SRAM_CE3_N = 1'b0;
		
	assign SRAM_ADSC_N = 1'b1;
	assign SRAM_ADSP_N = 1'b1;
	assign SRAM_ADV_N = 1'b0;
	
	assign SRAM_BE_N0 = 1'b1;
	assign SRAM_BE_N1 = 1'b1;
	assign SRAM_BE_N2 = 1'b1;
	assign SRAM_BE_N3 = 1'b1;
	
	assign SRAM_DPA0 = 1'b0;
	assign SRAM_DPA1 = 1'b0; 
	assign SRAM_DPA2 = 1'b0;
	assign SRAM_DPA3 = 1'b0;
	
	assign SRAM_WE_N = 1'b1;
	
// Shared Cache (SSRAM) controller snooping
// ---------------------------------------
	SharedMemoryController sharedCacheControl
	(cpu0sramad, cpu0cacherd, cpu0controlwd, cpu0blockstatus, 
		cpu0ad2, cpu0ad2rd, cpu0ad2tag, cpu0ad2blockstatus, 
		cpu0ad3, cpu0ad3rd, cpu0ad3tag, cpu0ad3blockstatus, 
		cpu0ad4, cpu0ad4rd, cpu0ad4tag, cpu0ad4blockstatus, 
		cpu0memorywe, cpu0memoryfunct, cpu0hit, 
		cpu0fetchblock, cpu0invalidate,
		
		cpu1sramad, cpu1cacherd, cpu1controlwd, cpu1blockstatus, 
		cpu1ad2, cpu1ad2rd, cpu1ad2tag, cpu1ad2blockstatus, 
		cpu1ad3, cpu1ad3rd, cpu1ad3tag, cpu1ad3blockstatus, 
		cpu1ad4, cpu1ad4rd, cpu1ad4tag, cpu1ad4blockstatus, 
		cpu1memorywe, cpu1memoryfunct, cpu1hit, 
		cpu1fetchblock, cpu1invalidate,
		
		cpu2sramad, cpu2cacherd, cpu2controlwd, cpu2blockstatus, 
		cpu2ad2, cpu2ad2rd, cpu2ad2tag, cpu2ad2blockstatus, 
		cpu2ad3, cpu2ad3rd, cpu2ad3tag, cpu2ad3blockstatus, 
		cpu2ad4, cpu2ad4rd, cpu2ad4tag, cpu2ad4blockstatus, 
		cpu2memorywe, cpu2memoryfunct, cpu2hit, 
		cpu2fetchblock, cpu2invalidate,
		
		cpu3sramad, cpu3cacherd, cpu3controlwd, cpu3blockstatus, 
		cpu3ad2, cpu3ad2rd, cpu3ad2tag, cpu3ad2blockstatus, 
		cpu3ad3, cpu3ad3rd, cpu3ad3tag, cpu3ad3blockstatus, 
		cpu3ad4, cpu3ad4rd, cpu3ad4tag, cpu3ad4blockstatus, 
		cpu3memorywe, cpu3memoryfunct, cpu3hit, 
		cpu3fetchblock, cpu3invalidate);

// Bus arbiter for SSRAM bus
// --------------------------------------
	logic [7:0] requests, grants;
	logic baenable;
	logic [1:0] Stall_counter, Stall_counter1;
	
	// Stall the round robin for 3 cycles if it is a read
	// and if a WB is required, disable the enable
	assign baenable = ~(((cpu0sramoe | cpu1sramoe | cpu2sramoe | cpu3sramoe)) & ((~Stall_counter[1] & ~Stall_counter[0])
		| (~Stall_counter[1] & Stall_counter[0]) | (Stall_counter[1] & ~Stall_counter[0])));
	
	initial Stall_counter = 2'd0;

	assign Stall_counter1 = Stall_counter + 1;
	
	always @(posedge new_clock) begin
		if (~baenable) Stall_counter <= Stall_counter1; // Add reset value
		else if (baenable) Stall_counter <= 2'd0;
	end
	
	assign requests[7] = cpu0sramgw;
	assign requests[6] = cpu0sramoe;
	assign requests[5] = cpu1sramgw;
	assign requests[4] = cpu1sramoe;
	
	assign requests[3] = cpu2sramgw;
	assign requests[2] = cpu2sramoe;
	assign requests[1] = cpu3sramgw;
	assign requests[0] = cpu3sramoe;
	
	BusArbiter8way SSRAMBusArbiter(new_clock, baenable, 1'b0, requests, grants);
	
	logic [31:0] SRAM_DQ;
	assign SRAM_A =  (grants[7] | grants[6]) ? cpu0sramad : ((grants[5] | grants[4]) ? cpu1sramad : ((grants[3] | grants[2]) ? cpu2sramad : 
									((grants[1] | grants[0]) ? cpu3sramad : 6'd0)));
	
	assign SRAM_GW_N = grants[7] ? ~cpu0sramgw : (grants[5] ? ~cpu1sramgw : (grants[3] ? ~cpu2sramgw : (grants[1] ? ~cpu3sramgw : 1'b1)));
										 
	assign SRAM_OE_N = grants[6] ? ~cpu0sramoe : (grants[4] ? ~cpu1sramoe : (grants[2] ? ~cpu2sramoe : (grants[0] ? ~cpu3sramoe : 1'b1)));
	
	assign SRAM_DQ = ~SRAM_GW_N ? grants[7] ? cpu0sramwd : (grants[5] ? cpu1sramwd : (grants[3] ?  cpu2sramwd :
																(grants[1] ? cpu3sramwd : 32'hdeadbeef))) : 32'dz;
																
	logic [31:0] SRAMRD;
	assign cpu0sramrd  = SRAMRD;
	assign cpu1sramrd  = SRAMRD;// if on board, -> SRAM_DQ
	assign cpu2sramrd  = SRAMRD;
	assign cpu3sramrd  = SRAMRD;
	assign cpu0stallMe = (cpu0sramgw & ~grants[7]) | (cpu0sramoe & ~grants[6]) | (cpu0sramoe & ~baenable);// | (cpu0sramoe & bareset);
	assign cpu1stallMe = (cpu1sramgw & ~grants[5]) | (cpu1sramoe & ~grants[4]) | (cpu1sramoe & ~baenable);// | (cpu1sramoe & bareset);
	assign cpu2stallMe = (cpu2sramgw & ~grants[3]) | (cpu2sramoe & ~grants[2]) | (cpu2sramoe & ~baenable);// | (cpu2sramoe & bareset);
	assign cpu3stallMe = (cpu3sramgw & ~grants[1]) | (cpu3sramoe & ~grants[0]) | (cpu3sramoe & ~baenable);// | (cpu3sramoe & bareset);
	
// Shared RAM module
// --------------------------------------
RAM #(6) sharedRAM(SRAM_A[5:0], SRAM_DQ, new_clock, ~SRAM_GW_N, SRAMRD);	
	
// CPU 0
// ---------------------------------------
	CPU0 cpu0(new_clock, cpu0ledr, cpu0ledg, SW, 
		cpu0hex0, cpu0hex1, cpu0hex2, cpu0hex3, 
		cpu0hex4, cpu0hex5, cpu0hex6, cpu0hex7,
		cpu0sramgw, cpu0sramoe, cpu0sramad, 
		cpu0sramrd, cpu0sramwd, cpu0cacherd, //cpu0cachewd,
		cpu0stallMe, /*cpu0ramStall,	*/cpu0controlwd, cpu0blockstatus, 
		cpu0ad2, cpu0ad2rd, cpu0ad2tag, cpu0ad2blockstatus, 
		cpu0ad3, cpu0ad3rd, cpu0ad3tag, cpu0ad3blockstatus, 
		cpu0ad4, cpu0ad4rd, cpu0ad4tag, cpu0ad4blockstatus, 
		cpu0memorywe, cpu0memoryfunct, cpu0hit, 
		cpu0fetchblock, cpu0invalidate);
		
// CPU 1
// ---------------------------------------
	CPU1 cpu1(new_clock, cpu1ledr, cpu1ledg, SW, 
		cpu1hex0, cpu1hex1, cpu1hex2, cpu1hex3, 
		cpu1hex4, cpu1hex5, cpu1hex6, cpu1hex7,
		cpu1sramgw, cpu1sramoe, cpu1sramad, 
		cpu1sramrd, cpu1sramwd, cpu1cacherd, //cpu1cachewd, 
		cpu1stallMe, /*cpu1ramStall,	*/cpu1controlwd, cpu1blockstatus, 
		cpu1ad2, cpu1ad2rd, cpu1ad2tag, cpu1ad2blockstatus, 
		cpu1ad3, cpu1ad3rd, cpu1ad3tag, cpu1ad3blockstatus, 
		cpu1ad4, cpu1ad4rd, cpu1ad4tag, cpu1ad4blockstatus, 
		cpu1memorywe, cpu1memoryfunct, cpu1hit, 
		cpu1fetchblock, cpu1invalidate);
		
// CPU 2
// ---------------------------------------
	CPU2 cpu2(new_clock, cpu2ledr, cpu2ledg, SW, 
		cpu2hex0, cpu2hex1, cpu2hex2, cpu2hex3, 
		cpu2hex4, cpu2hex5, cpu2hex6, cpu2hex7,
		cpu2sramgw, cpu2sramoe, cpu2sramad, 
		cpu2sramrd, cpu2sramwd, cpu2cacherd, //cpu2cachewd, 
		cpu2stallMe, /*cpu2ramStall,	*/cpu2controlwd, cpu2blockstatus, 
		cpu2ad2, cpu2ad2rd, cpu2ad2tag, cpu2ad2blockstatus, 
		cpu2ad3, cpu2ad3rd, cpu2ad3tag, cpu2ad3blockstatus, 
		cpu2ad4, cpu2ad4rd, cpu2ad4tag, cpu2ad4blockstatus, 
		cpu2memorywe, cpu2memoryfunct, cpu2hit, 
		cpu2fetchblock, cpu2invalidate);
		
// CPU 3
// ---------------------------------------
	CPU3 cpu3(new_clock, cpu3ledr, cpu3ledg, SW, 
		cpu3hex0, cpu3hex1, cpu3hex2, cpu3hex3, 
		cpu3hex4, cpu3hex5, cpu3hex6, cpu3hex7,
		cpu3sramgw, cpu3sramoe, cpu3sramad, 
		cpu3sramrd, cpu3sramwd, cpu3cacherd, //cpu3cachewd, 
		cpu3stallMe, /*cpu3ramStall,	*/cpu3controlwd, cpu3blockstatus, 
		cpu3ad2, cpu3ad2rd, cpu3ad2tag, cpu3ad2blockstatus, 
		cpu3ad3, cpu3ad3rd, cpu3ad3tag, cpu3ad3blockstatus, 
		cpu3ad4, cpu3ad4rd, cpu3ad4tag, cpu3ad4blockstatus, 
		cpu3memorywe, cpu3memoryfunct, cpu3hit, 
		cpu3fetchblock, cpu3invalidate);
	
// Keys for changing displays on board
// ---------------------------------------	
	logic [3:0] updatedKeys;
	logic keyUpdate;
	assign keyUpdate = ~((KEY[3] & KEY[2] & KEY[1] & KEY[0]));
	Register #(4) keys(~KEY, new_clock, keyUpdate, updatedKeys);
	
	assign LEDR = updatedKeys[0] ? cpu0ledr : (updatedKeys[1] ? cpu1ledr : 18'd0);                                                           
	assign LEDG[7:0] = updatedKeys[0] ? cpu0ledg : (updatedKeys[1] ? cpu1ledg : 7'd0);
	assign HEX0 = updatedKeys[0] ? cpu0hex0 : (updatedKeys[1] ? cpu1hex0 : 8'b11111111);
	assign HEX1 = updatedKeys[0] ? cpu0hex1 : (updatedKeys[1] ? cpu1hex1 : 8'b11111111);
	assign HEX2 = updatedKeys[0] ? cpu0hex2 : (updatedKeys[1] ? cpu1hex2 : 8'b11111111);
	assign HEX3 = updatedKeys[0] ? cpu0hex3 : (updatedKeys[1] ? cpu1hex3 : 8'b11111111);
	assign HEX4 = updatedKeys[0] ? cpu0hex4 : (updatedKeys[1] ? cpu1hex4 : 8'b11111111);
	assign HEX5 = updatedKeys[0] ? cpu0hex5 : (updatedKeys[1] ? cpu1hex5 : 8'b11111111);
	assign HEX6 = updatedKeys[0] ? cpu0hex6 : (updatedKeys[1] ? cpu1hex6 : 8'b11111111);
	assign HEX7 = updatedKeys[0] ? cpu0hex7 : (updatedKeys[1] ? cpu1hex7 : 8'b11111111);
	
endmodule 