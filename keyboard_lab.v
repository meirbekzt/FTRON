module keyboard_lab(LEDR, HEX0, HEX1, SW, KEY, CLOCK_50, PS2_DAT, PS2_CLK);
	input [9:0] SW;
	input CLOCK_50;
	input PS2_DAT;
	input PS2_CLK;
	input [3:0] KEY;
	output [9:0] LEDR;
	output [6:0] HEX0;
	output [6:0] HEX1;

	wire reset;
	assign reset = ~KEY[0];

	wire clk;
	assign clk = CLOCK_50;

	wire valid, makeBreak;
	wire [7:0] outCode;


	keyboard_press_driver keyboard_press_driver(
		.CLOCK_50(CLOCK_50),
		.valid(valid),
		.makeBreak(makeBreak),
		.outCode(outCode),
		.reset(reset),
		.PS2_DAT(PS2_DAT),
		.PS2_CLK(PS2_CLK),
	);

	always @(*) begin
		outCode == 8'd75;
	end 

	hex_decoder hex1(
		.hex_digit(outCode[7:4]),
	   	.segments(HEX1)
		);

	hex_decoder hex0(
		.hex_digit(outCode[3:0]),
	   	.segments(HEX0)
		);


endmodule
