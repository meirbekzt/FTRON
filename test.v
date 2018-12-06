`include "n_counter.v"
`include "ram128x256.v"

module test(resetn, CLOCK_50, outCode);
    input resetn;
    input CLOCK_50;
    input [7:0] outCode;

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
	
	reg game;

	// wires that go from datapath to control
	wire kill;
	wire clr_screen_finish;

	wire [1:0] p1_dir;
	wire [1:0] p2_dir;
	reg [1:0] p1_n_dir;
	reg [1:0] p2_n_dir;

	always @(*) begin
		game = 1'b0;
		if (outCode == 8'h29)
			begin
			game = 1'b1;
			if (init_game || init_screen) begin
				p1_n_dir = 2'b11;
				p2_n_dir = 2'b10;
				end
			end
	end
	assign p1_dir = p1_n_dir;
	assign p2_dir = p2_n_dir;
	wire go;
	assign go = game;
	wire [1:0] select_player;
	
	// Instantiate datapath
	datapath d0(
		.plot(writeEn),
		.resetn(resetn),
		.clk(CLOCK_50),
		.p1_dir(p1_dir),
		.p2_dir(p2_dir),
		.cur_x(x),
		.cur_y(y),
		.update(update),
		.init_game(init_game),
		.init_screen(init_screen),
		.kill(kill),
		.clr_screen(clr_screen),
		.select_player(select_player),
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
		.select_player(select_player),
		.select_mem_data(select_mem_data),
		.select_mem_addr(select_mem_addr),
		.mem_ctr(mem_ctr),
		.mem_write(mem_write),
		.check_collision(check_collision)
		);
endmodule

module datapath(plot, resetn, clk, cur_x, cur_y, update, init_game, init_screen, clr_screen, select_player, mem_write, p1_dir, p2_dir, kill, clr_screen_finish, select_mem_data, select_mem_addr, mem_ctr, check_collision);
	input plot;
	input resetn;
	input clk;
	input update;
	input [1:0] p1_dir;
	input [1:0] p2_dir;
	input [1:0] select_player;
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
	
	localparam 	S_P1_UP = 3'd0,
			    S_P1_RIGHT = 3'd1,
				S_P1_DOWN = 3'd2,
				S_P1_LEFT = 3'd3,
				S_P2_UP = 3'd4,
			    S_P2_RIGHT = 3'd5,
				S_P2_DOWN = 3'd6,
				S_P2_LEFT = 3'd7,
                TOP_EDGE = 7'd1,
                BOTTOM_EDGE = 7'd118,
                LEFT_EDGE = 8'd1,
                RIGHT_EDGE = 8'd158,
				P1_START_X = 8'd40,
				P1_START_Y = 7'd60,
				P2_START_X = 8'd100,
				P2_START_Y = 7'd60;
				
	// data registers
	reg [7:0] next_x;
	reg [6:0] next_y;
	reg [7:0] p1_next_x;
	reg [6:0] p1_next_y;
	reg [7:0] p2_next_x;
	reg [6:0] p2_next_y;
	// direction registers, 1 if going right, 0 if left
	// 1 if going up, 0 if going down
	
	reg [2:0] p1_cur_dir, p1_next_dir;
	reg [2:0] p2_cur_dir, p2_next_dir;
	
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
	begin: p1_state_table
		case (p1_cur_dir)
			S_P1_UP: p1_next_dir = p1_dir[1] ? (p1_dir[0] ? S_P1_RIGHT : S_P1_LEFT) : S_P1_UP;
			S_P1_RIGHT: p1_next_dir = p1_dir[1] ? S_P1_RIGHT : (p1_dir[0] ? S_P1_UP : S_P1_DOWN);
			S_P1_DOWN: p1_next_dir = p1_dir[1] ? (p1_dir[0] ? S_P1_RIGHT : S_P1_LEFT) : S_P1_DOWN;
			S_P1_LEFT: p1_next_dir = p1_dir[1] ? S_P1_LEFT : (p1_dir[0] ? S_P1_UP : S_P1_DOWN);
			default: p1_next_dir = S_P1_UP;
		endcase
	end
	
	always @(*)
	begin: p2_state_table
		case (p2_cur_dir)
			S_P2_UP: p2_next_dir = p2_dir[1] ? (p2_dir[0] ? S_P2_RIGHT : S_P2_LEFT) : S_P2_UP;
			S_P2_RIGHT: p2_next_dir = p2_dir[1] ? S_P2_RIGHT : (p2_dir[0] ? S_P2_UP : S_P2_DOWN);
			S_P2_DOWN: p2_next_dir = p2_dir[1] ? (p2_dir[0] ? S_P2_RIGHT : S_P2_LEFT) : S_P2_DOWN;
			S_P2_LEFT: p2_next_dir = p2_dir[1] ? S_P2_LEFT : (p2_dir[0] ? S_P2_UP : S_P2_DOWN);
			default: p2_next_dir = S_P2_UP;
		endcase
	end
	
	always @(posedge clk) begin
		if (init_screen) begin
			cur_x <= 8'd1;
			cur_y <= 7'd1;
			next_x <= 8'd1;
			next_y <= 7'd1;
			p1_cur_dir <= S_P1_RIGHT;
			p2_cur_dir <= S_P2_LEFT;
            kill <= 1'b0;
			clr_screen_finish <= 1'b0;
		end
	
		else if (init_game) begin
			// init_gamealize all values
			p1_next_x <= P1_START_X;
			p1_next_y <= P1_START_Y;
			p2_next_x <= P2_START_X;
			p2_next_y <= P2_START_Y;
			p1_cur_dir <= S_P1_RIGHT;
			p2_cur_dir <= S_P2_LEFT;
			kill <= 1'b0;
			clr_screen_finish <= 1'b0;
		end
			
		else if (plot && ~kill) begin
			if (clr_screen) begin
				if (next_y == 7'd11) 
					clr_screen_finish <= 1'b1;

				else begin
					cur_x <= next_x;
					cur_y <= next_y;

					if (cur_x == 8'd15) begin
						next_x <= 1'b1;
						next_y <= next_y + 1'b1;
					end

					else
						next_x <= next_x + 1'b1;
				end
			end
			
			else begin
				cur_x <= select_player[0] == 1'b0 ? p1_next_x : p2_next_x;
				cur_y <= select_player[0] == 1'b0 ? p1_next_y : p2_next_y;
			end
		end

		else if (update) begin
			p1_cur_dir <= p1_next_dir;
			case (p1_cur_dir)
				S_P1_UP: begin 
						p1_next_y <= p1_next_y - 1'b1;
						end
				S_P1_RIGHT: begin 
						p1_next_x <= p1_next_x + 1'b1;
						end
				S_P1_LEFT: begin 
						p1_next_x <= p1_next_x - 1'b1;
						end
				S_P1_DOWN: begin 
						p1_next_y <= p1_next_y + 1'b1;
						end
			endcase
			
			p2_cur_dir <= p2_next_dir;
			case (p2_cur_dir)
				S_P2_UP: begin 
						p2_next_y <= p2_next_y - 1'b1;
						end
				S_P2_RIGHT: begin 
						p2_next_x <= p2_next_x + 1'b1;
						end
				S_P2_LEFT: begin 
						p2_next_x <= p2_next_x - 1'b1;
						end
				S_P2_DOWN: begin 
						p2_next_y <= p2_next_y + 1'b1;
						end
			endcase
			
		end

//		else if (0 && check_collision) begin
//			if (((next_y == TOP_EDGE && current_dir == S_UP) || (next_y == BOTTOM_EDGE && current_dir == S_DOWN)
//			|| (next_x == RIGHT_EDGE && current_dir == S_RIGHT) || (next_x == LEFT_EDGE && current_dir == S_LEFT))) 
//				kill <= 1'b1;
//
//			if (|(row & (256'b1 << next_x)))
//					kill <= 1'b1;
//		end
	end
endmodule


module control(clk, resetn, go, plot, update, clr_screen, init_game, init_screen, colour_out, kill, clr_screen_finish, select_player, mem_write, select_mem_data, select_mem_addr, mem_ctr, check_collision);
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
	output reg [1:0] select_player;
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
					S_DRAW_P1		= 4'd3,
					S_DRAW_FINISH_P1 = 4'd4,
					S_DRAW_P2 = 4'd5,
					S_DRAW_FINISH_P2 = 4'd6,
					S_WAIT				= 4'd7,
					S_UPDATE		=4'd8,
					S_CHECK_COLLISION_1 = 4'd9,
					S_CHECK_COLLISION_2 = 4'd10,
                    S_KILL          = 4'd11,
					BG_COLOUR 	= 3'b000,
					PLAYER_1_COLOUR = 3'b010,
					PLAYER_2_COLOUR = 3'b011;

	n_counter #(19) delay_counter(
		.clk(clk),
		.resetn(reset_frame),
		.d(4), // value should be 833334
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
			S_SETUP: next_state = ~|mem_ctr ? S_DRAW_P1 : S_SETUP;
			S_DRAW_P1: next_state = S_DRAW_FINISH_P1;
			S_DRAW_FINISH_P1: next_state = S_DRAW_P2;
			S_DRAW_P2: next_state = S_DRAW_FINISH_P2;
			S_DRAW_FINISH_P2: next_state = S_WAIT;
			S_WAIT: next_state = ~|frame_ctr ? S_UPDATE : S_WAIT;
			S_UPDATE: next_state = S_CHECK_COLLISION_1;
			S_CHECK_COLLISION_1: next_state = S_CHECK_COLLISION_2;
			S_CHECK_COLLISION_2: next_state = kill ? S_KILL : S_DRAW_P1;
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
		select_player = 2'b00;
		colour_out = 3'b000;
		
		
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
			
			S_DRAW_P1: begin
				plot = 1'b1;
				colour_out = PLAYER_1_COLOUR;
				mem_write = 1'b1;
			end

			S_DRAW_FINISH_P1: begin
				plot = 1'b1;
				colour_out = PLAYER_1_COLOUR;
				mem_write = 1'b1;
			end
			
			S_DRAW_P2: begin
				plot = 1'b1;
				colour_out = PLAYER_2_COLOUR;
				mem_write = 1'b1;
				select_player = 2'b01;
			end

			S_DRAW_FINISH_P2: begin
				plot = 1'b1;
				colour_out = PLAYER_2_COLOUR;
				mem_write = 1'b1;
				select_player = 2'b01;
			end

			S_WAIT: begin
				reset_frame = 1'b1;
			end
			
			S_UPDATE: begin
				update = 1'b1;
			end
			
			S_CHECK_COLLISION_1: begin
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