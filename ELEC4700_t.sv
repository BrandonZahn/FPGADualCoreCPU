module ELEC4700_t;

	logic 				clock;
	logic [17:0] 	SW;
	logic  [3:0] 	KEY;
	logic [17:0] 	LEDR;
	logic  [8:0] 	LEDG;
	logic  [7:0] 	HEX0;
	logic  [7:0] 	HEX1;
	logic  [7:0] 	HEX2;
	logic  [7:0] 	HEX3;
	logic  [7:0] 	HEX4;
	logic  [7:0] 	HEX5;
	logic  [7:0] 	HEX6;
	logic  [7:0] 	HEX7;
	logic 		  	SRAM_CLK;
	logic 		  	SRAM_CE1_N, SRAM_CE2, SRAM_CE3_N;
	logic [18:0] 	SRAM_A;
	logic 		  	SRAM_ADSC_N, SRAM_ADSP_N, SRAM_ADV_N;
	logic 		  	SRAM_BE_N0, SRAM_BE_N1, SRAM_BE_N2, SRAM_BE_N3;
	logic 		  	SRAM_DPA0, SRAM_DPA1, SRAM_DPA2, SRAM_DPA3;
	logic 		  	SRAM_GW_N, SRAM_OE_N, SRAM_WE_N;

	ELEC4700 ELEC4700_t
	(
		clock, SW, KEY, LEDR, LEDG, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7, 
		SRAM_CLK, SRAM_CE1_N, SRAM_CE2, SRAM_CE3_N, SRAM_A, SRAM_ADSC_N, 
		SRAM_ADSP_N, SRAM_ADV_N, SRAM_BE_N0, SRAM_BE_N1, SRAM_BE_N2, SRAM_BE_N3, 
		SRAM_DPA0, SRAM_DPA1, SRAM_DPA2, SRAM_DPA3, SRAM_GW_N, SRAM_OE_N, SRAM_WE_N
	);
	
	initial SW = 17'd0;
	initial KEY = 4'd0;
	initial clock = 0;
	always #2 clock = ~clock;
endmodule 