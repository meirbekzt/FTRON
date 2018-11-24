module test(resetn, reset, go, CLOCK_50, outCode);
    input resetn;
    input reset;
    input go;
    input CLOCK_50;
    input outCode;

	// Create the colour, cur_x, cur_y and writeEn wires that are inputs to the controller.
	wire [2:0] colour_out;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	wire update;

	wire valid, makeBreak;
	wire [7:0] outCode;
	wire [1:0] dir;
	reg [1:0] n_dir;
	wire kill;

	always @(posedge CLOCK_50) begin
		if (outCode == 8'h75)
			n_dir <= 2'b01;
		if (outCode == 8'h74)
			n_dir <= 2'b11;
		if (outCode == 8'h72)
			n_dir <= 2'b00;
		if (outCode == 8'h6b)
			n_dir <= 2'b10;
	end
	assign dir = n_dir;

	// Instantiate datapath
	datapath d0(
		.plot(writeEn),
		.resetn(resetn),
		.clk(CLOCK_50),
		.dir(dir),
		.cur_x(x),
		.cur_y(y),
		.update(update),
		.kill(kill)
		);

    // Instansiate FSM control
    control c0(
		.clk(CLOCK_50),
		.resetn(resetn),
		.go(go),
		.kill(kill),
		.plot(writeEn),
		.update(update),
		.colour_out(colour_out)
		);
endmodule
