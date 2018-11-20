
// Part 2 skeleton

module part3
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
        KEY, SW, // The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		PS2_DAT,
		PS2_CLK
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;
	input PS2_DAT;
	input PS2_CLK;


	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	wire resetn;
	assign resetn = KEY[0];
	wire reset;
	assign reset = ~KEY[3];

	wire go;
	assign go = ~KEY[1];
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [2:0] colour_out;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	wire [1:0] x_ctr;
	wire [1:0] y_ctr;
	wire update;

	assign colour = SW[9:7];

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour_out),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	wire valid, makeBreak;
	wire [7:0] outCode;
	wire [1:0] dir;
	reg [1:0] n_dir;

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
		if (outCode == 8'h75)
			n_dir = 2'b01;
		if (outCode == 8'h74)
			n_dir = 2'b11;
		if (outCode == 8'h72)
			n_dir = 2'b00;
		if (outCode == 8'h6b)
			n_dir = 2'b10;
	end
	assign dir = n_dir;
    
	// Instantiate datapath
	datapath d0(
		.plot(writeEn),
		.x_ctr(x_ctr),
		.y_ctr(y_ctr),
		.resetn(resetn),
		.clk(CLOCK_50),
		.dir(dir),
		.x_out(x),
		.y_out(y),
		.update(update)
		);

    // Instansiate FSM control
    control c0(
		.clk(CLOCK_50),
		.resetn(resetn),
		.go(go),
		.plot(writeEn),
		.draw_x(x_ctr),
		.draw_y(y_ctr),
		.update(update),
		.colour(colour),
		.colour_out(colour_out)
		);
endmodule

module datapath(plot, x_ctr, y_ctr, resetn, clk, x_out, y_out, update, dir);
	input plot;
	input resetn;
	input clk;
	input [1:0] x_ctr;
	input [1:0] y_ctr;
	input update;
	input [1:0] dir;
	
	output reg [7:0] x_out;
	output reg [6:0] y_out;
	
	localparam 	S_UP = 3'd0,
			    S_RIGHT = 3'd1,
				S_DOWN = 3'd2,
				S_LEFT = 3'd3;
	
	// data registers
	reg [7:0] x;
	reg [6:0] y;
	// direction registers, 1 if going right, 0 if left
	// 1 if going up, 0 if going down
	
	reg [2:0] current_dir, next_dir;
	
	always @(*)
	begin: state_table
		case (current_dir)
			S_UP: next_dir = dir[1] ? (dir[0] ? S_RIGHT : S_LEFT) : S_UP;
			S_RIGHT: next_dir = dir[1] ? S_RIGHT : (dir[0] ? S_UP : S_DOWN);
			S_DOWN: next_dir = dir[1] ? (dir[0] ? S_RIGHT : S_LEFT) : S_DOWN;
			S_LEFT: next_dir = dir[1] ? S_LEFT : (dir[0] ? S_UP : S_DOWN);
			default: next_dir = S_UP;
		endcase
	end
	
	always @(posedge clk) begin
		if (~resetn) begin
			// initalize all values
			x_out <= 8'd0;
			y_out <= 7'd60;
            x <= 8'd0;
            y <= 7'd60;
			current_dir <= S_UP;
		end
			
		else begin
			if (plot) begin
				x_out <= x + x_ctr;
				y_out <= y + y_ctr;
			end

			if (update) begin
				current_dir <= next_dir;
				case (current_dir)
					S_UP: begin y <= y - 1'b1;
						   y_out <= y_out - 1'b1;
						   end
					S_RIGHT: begin x <= x + 1'b1;
						   x_out <= x_out + 1'b1;
						   end
					S_LEFT: begin x <= x - 1'b1;
						   x_out <= x_out - 1'b1;
						   end
					S_DOWN: begin y <= y + 1'b1;
						   y_out <= y_out + 1'b1;
						   end
				endcase
			end
		end
	end
endmodule

module control(clk, resetn, go, plot, draw_x, draw_y, update, colour, colour_out);
	input clk;
	input resetn;
	input go;
	input [2:0] colour;
	output reg [2:0] colour_out;


	output reg plot;
	output [1:0] draw_x;
	output [1:0] draw_y;
	output reg update;
	
	wire [3:0] grid_wire;	
	reg reset_frame;

	// wires for delay counter and frame counter
	wire [19:0] delay_ctr;
	wire [3:0] frame_ctr;

	reg [2:0] current_state, next_state;
	
	localparam		S_START				= 3'd0,
					S_DRAW		= 3'd1,
					S_DRAW_FINISH = 3'd2,
					S_WAIT				= 3'd3,
					S_UPDATE		=3'd4;
					
	
	four_bit_counter grid(
		.resetn(plot),
		.clk(clk),
		.enable(plot),
		.q(grid_wire)
	);

	assign draw_x = grid_wire[1:0];
	assign draw_y = grid_wire[3:2];	

	n_counter #(19) delay_counter(
		.clk(clk),
		.resetn(reset_frame),
		.d(833334), // value should be 833334
		.enable(1),
		.q(delay_ctr)
		);

	n_counter #(3) frame_counter(
		.clk(clk),
		.resetn(reset_frame),
		.d(3), // value should be 15
		.enable(~|delay_ctr),
		.q(frame_ctr)
		);

	always @(*)
	begin: state_table
		case (current_state)
			S_START: next_state = go ? S_DRAW : S_START;
			S_DRAW: next_state = &draw_x && &draw_y ? S_DRAW_FINISH : S_DRAW;
			S_DRAW_FINISH: next_state = S_WAIT;
			S_WAIT: next_state = ~|frame_ctr ? S_UPDATE : S_WAIT;
			S_UPDATE: next_state = S_DRAW;
			default: next_state = S_START;
		endcase
	end
	
	
	// Output logic
	always @(*) 
	begin: enable_signals
		plot = 1'b0;
		reset_frame = 1'b0;
		update = 1'b0;
		
		case (current_state)
			S_START: begin
			end

			S_DRAW: begin
				plot = 1'b1;
				colour_out = colour;
			end

			S_DRAW_FINISH: begin
				plot = 1'b1;
				colour_out = colour;
			end

			S_WAIT: begin
				reset_frame = 1'b1;
			end
			
			S_UPDATE: begin
				update = 1'b1;
			end
		endcase
	end
	
	always@(posedge clk)
    begin: state_FFs
        if(!resetn)
            current_state <= S_START;
        else
            current_state <= next_state;
    end // state_FFS
endmodule


module four_bit_counter(resetn, clk, enable, q);
	input resetn;
	input clk;
	input enable;
	output reg [3:0] q;
	
	
	always @(posedge clk) begin
		if (!resetn) 
			q <= 0;
		else if (enable) 
			q <= q + 1;
	end
endmodule

