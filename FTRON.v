module FTRON
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

	reg game;
//	assign go = ~KEY[1];
	// Create the colour, cur_x, cur_y and writeEn wires that are inputs to the controller.
	wire [2:0] colour_out;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;

	// wires that go from control to datapath
	wire update;
	wire init_game;
	wire init_screen;
	wire clr_screen;
	wire mem_write;
	wire select_mem_data;
	wire select_mem_addr;
	wire [6:0] mem_ctr;
	wire check_collision;

	// wires that go from datapath to control
	wire kill;
	wire clr_screen_finish;
	wire reset;
	assign reset = ~KEY[3];

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the init_gameial background
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
	defparam VGA.BACKGROUND_IMAGE = "display.mif";
			
	// keyboard wires
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
		.PS2_CLK(PS2_CLK)
	);
	
	always @(*) begin
		game = 1'b0;
		if (outCode == 8'h29 && valid && makeBreak)
			begin
			game = 1'b1;
			n_dir = 2'b11;
			end
		if (outCode == 8'h75 && valid && makeBreak)
			begin
			n_dir = 2'b01;
			end
		if (outCode == 8'h74 && valid && makeBreak)
			begin
			n_dir = 2'b11;
			end
		if (outCode == 8'h72 && valid && makeBreak)
			begin
			n_dir = 2'b00;
			end
		if (outCode == 8'h6b && valid && makeBreak)
			begin
			n_dir = 2'b10;
			end
	end
	assign dir = n_dir;
   wire go;
	assign go = game;
	
	// Instantiate datapath
	datapath d0(
		.plot(writeEn),
		.resetn(resetn),
		.clk(CLOCK_50),
		.dir(dir),
		.cur_x(x),
		.cur_y(y),
		.update(update),
		.init_game(init_game),
		.init_screen(init_screen),
		.kill(kill),
		.clr_screen(clr_screen),
		.clr_screen_finish(clr_screen_finish),
		.select_mem_data(select_mem_data),
		.select_mem_addr(select_mem_addr),
		.mem_write(mem_write),
		.mem_ctr(mem_ctr),
		.check_collision(check_collision)
		);

    // Instansiate FSM control
    control c0(
		.clk(CLOCK_50),
		.resetn(resetn),
		.go(go),
		.kill(kill),
		.plot(writeEn),
		.update(update),
		.init_game(init_game),
		.init_screen(init_screen),
		.colour_out(colour_out),
		.clr_screen(clr_screen),
		.clr_screen_finish(clr_screen_finish),
		.select_mem_data(select_mem_data),
		.select_mem_addr(select_mem_addr),
		.mem_ctr(mem_ctr),
		.mem_write(mem_write),
		.check_collision(check_collision)
		);
endmodule

module datapath(plot, resetn, clk, cur_x, cur_y, update, init_game, init_screen, clr_screen, mem_write, dir, kill, clr_screen_finish, select_mem_data, select_mem_addr, mem_ctr, check_collision);
	input plot;
	input resetn;
	input clk;
	input update;
	input [1:0] dir;
	input clr_screen; 
	input init_game;
	input init_screen;
	input mem_write;
	input select_mem_data;
	input select_mem_addr;
	input [6:0] mem_ctr;
	input check_collision;
	
    output reg kill;
	output reg clr_screen_finish;
	output reg [7:0] cur_x;
	output reg [6:0] cur_y;
	
	localparam 	S_UP = 3'd0,
			    S_RIGHT = 3'd1,
				S_DOWN = 3'd2,
				S_LEFT = 3'd3,
                TOP_EDGE = 7'd1,
                BOTTOM_EDGE = 7'd118,
                LEFT_EDGE = 8'd1,
                RIGHT_EDGE = 8'd158,
				START_X = 8'd40,
				START_Y = 7'd60;
				
	// data registers
	reg [7:0] next_x;
	reg [6:0] next_y;
	// direction registers, 1 if going right, 0 if left
	// 1 if going up, 0 if going down
	
	reg [1:0] current_dir, next_dir;
	
	// components for RAM function
	wire [255:0] row;
	reg [255:0] mem_data_in;
	reg [6:0] mem_addr_in;

	ram128x256 grid(
		.address(mem_addr_in),
		.clock(clk),
		.data(mem_data_in),
		.wren(mem_write),
		.q(row)
	);

   // Muxes for address and data inputs to memory
   always @(*) begin
	   if (select_mem_data) 
		   mem_data_in = 256'b0;
	   else 
		   mem_data_in = row | 256'b1 << next_x;

	   if (select_mem_addr)
		   mem_addr_in = mem_ctr;
	   else
		   mem_addr_in = next_y;
   end

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
		if (init_screen) begin
			cur_x <= 8'd1;
			cur_y <= 7'd1;
			next_x <= 8'd1;
			next_y <= 7'd1;
			current_dir <= S_RIGHT;
            kill <= 1'b0;
			clr_screen_finish <= 1'b0;
		end
	
		else if (init_game) begin
			// init_gamealize all values
			cur_x <= START_X;
			cur_y <= START_Y;
			next_x <= START_X;
			next_y <= START_Y;
			current_dir <= S_RIGHT;
			kill <= 1'b0;
			clr_screen_finish <= 1'b0;
		end
			
		else if (plot && ~kill) begin
			if (clr_screen) begin
				if (next_y == 7'd119) 
					clr_screen_finish <= 1'b1;

				else begin
					cur_x <= next_x;
					cur_y <= next_y;

					if (cur_x == 8'd157) begin
						next_x <= 1'b1;
						next_y <= next_y + 1'b1;
					end

					else
						next_x <= next_x + 1'b1;
				end
			end
			
			else begin
				cur_x <= next_x;
				cur_y <= next_y;
			end
		end

		else if (update) begin
			current_dir <= next_dir;
			case (current_dir)
				S_UP: begin 
						next_y <= next_y - 1'b1;
						end
				S_RIGHT: begin 
						next_x <= next_x + 1'b1;
						end
				S_LEFT: begin 
						next_x <= next_x - 1'b1;
						end
				S_DOWN: begin 
						next_y <= next_y + 1'b1;
						end
			endcase
			
		end

		else if (check_collision) begin
			if (((next_y == TOP_EDGE && current_dir == S_UP) || (next_y == BOTTOM_EDGE && current_dir == S_DOWN)
			|| (next_x == RIGHT_EDGE && current_dir == S_RIGHT) || (next_x == LEFT_EDGE && current_dir == S_LEFT))) 
				kill <= 1'b1;

			if (|(row & (256'b1 << next_x)))
					kill <= 1'b1;
		end
	end
endmodule


module control(clk, resetn, go, plot, update, clr_screen, init_game, init_screen, colour_out, kill, clr_screen_finish, mem_write, select_mem_data, select_mem_addr, mem_ctr, check_collision);
	input clk;
	input resetn;
	input go;
    input kill;
	input clr_screen_finish;

	output reg [2:0] colour_out;
	// enable signals
	output reg plot;
	output reg update;
	output reg clr_screen;
	output reg init_game;
	output reg init_screen;
	output reg mem_write;
	output reg select_mem_data;
	output reg select_mem_addr;
	output reg check_collision;
	output [6:0] mem_ctr;
	
	reg [2:0] colour;
	
	// internal regs 
	reg reset_frame;
	// wires for all counter 
	wire [19:0] delay_ctr;
	wire [3:0] frame_ctr;

	reg [3:0] current_state, next_state;
	
	parameter player = 1;
	always@(*)
	begin
		if (player == 1) begin
			colour = 3'b010;
			end
		else begin
			colour = 3'b011;
			end
	end
	
	localparam		S_START				= 4'd0,
					S_CLEAR				= 4'd1,
					S_SETUP				= 4'd2,
					S_DRAW		= 4'd3,
					S_DRAW_FINISH = 4'd4,
					S_WAIT				= 4'd5,
					S_UPDATE		=4'd6,
					S_CHECK_COLLISION_1 = 4'd7,
					S_CHECK_COLLISION_2 = 4'd8,
                    S_KILL          = 4'd9,
					BG_COLOUR 	= 3'b000,
					PLAYER_1_COLOUR = 3'b010,
					PLAYER_2_COLOUR = 3'b011;

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

	n_counter #(6) mem_counter(
		.clk(clk),
		.resetn(mem_write),
		.d(7'd119),
		.enable(1),
		.q(mem_ctr)
	);

	always @(*)
	begin: state_table
		case (current_state)
			S_START: next_state = go ? S_CLEAR : S_START;
			S_CLEAR: next_state =  clr_screen_finish ? S_SETUP : S_CLEAR;
			S_SETUP: next_state = ~|mem_ctr ? S_DRAW : S_SETUP;
			S_DRAW: next_state = S_DRAW_FINISH;
			S_DRAW_FINISH: next_state = S_WAIT;
			S_WAIT: next_state = ~|frame_ctr ? S_UPDATE : S_WAIT;
			S_UPDATE: next_state = S_CHECK_COLLISION_1;
			S_CHECK_COLLISION_1: next_state = S_CHECK_COLLISION_2;
			S_CHECK_COLLISION_2: next_state = kill ? S_KILL : S_DRAW;
            S_KILL: next_state = S_START;
			default: next_state = S_START;
		endcase
	end
	
	
	// Output logic
	always @(*) 
	begin: enable_signals
		plot = 1'b0;
		reset_frame = 1'b0;
		clr_screen = 1'b0;
		update = 1'b0;
		init_game = 1'b0;
		init_screen = 1'b0;
		mem_write = 1'b0;
		select_mem_data = 1'b0;
		select_mem_addr = 1'b0;
		check_collision = 1'b0;
		
		case (current_state)
			S_START: begin
				init_screen = 1'b1;
			end
			
			S_CLEAR: begin
				plot = 1'b1;
				clr_screen = 1'b1;
				colour_out = BG_COLOUR;
			end

			S_SETUP: begin
				init_game = 1'b1;
				mem_write = 1'b1;
				select_mem_data = 1'b1;
				select_mem_addr = 1'b1;
			end
			
			S_DRAW: begin
				plot = 1'b1;
				colour_out = PLAYER_1_COLOUR;
				mem_write = 1'b1;
			end

			S_DRAW_FINISH: begin
				plot = 1'b1;
				colour_out = PLAYER_1_COLOUR;
				mem_write = 1'b1;
			end

			S_WAIT: begin
				reset_frame = 1'b1;
			end
			
			S_UPDATE: begin
				update = 1'b1;
			end
			
			S_CHECK_COLLISION_1: begin
				plot = 1'b1;
			end

			S_CHECK_COLLISION_2: begin
				check_collision = 1'b1;
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
