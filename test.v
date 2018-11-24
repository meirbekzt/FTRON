`include "n_counter.v"
module test(resetn, go, CLOCK_50, outCode);
    input resetn;
    input go;
    input CLOCK_50;
    input [7:0] outCode;

	// Create the colour, cur_x, cur_y and writeEn wires that are inputs to the controller.
	wire [2:0] colour_out;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	wire update;

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

module datapath(plot, resetn, clk, cur_x, cur_y, update, dir, kill);
	input plot;
	input resetn;
	input clk;
	input update;
	input [1:0] dir;
	
    output reg kill;
	output reg [7:0] cur_x;
	output reg [6:0] cur_y;
	
	localparam 	S_UP = 3'd0,
			    S_RIGHT = 3'd1,
				S_DOWN = 3'd2,
				S_LEFT = 3'd3,
                TOP_EDGE = 7'd0,
                BOTTOM_EDGE = 7'd119,
                LEFT_EDGE = 8'd0,
                RIGHT_EDGE = 8'd159,
				START_X = 8'd40,
				START_Y = 7'd60;
	
	// registers for drawing border coordinates
	reg [7:0] border_x;
	reg [6:0] border_y;
	
	// data registers
	reg [7:0] next_x;
	reg [6:0] next_y;
	// direction registers, 1 if going right, 0 if left
	// 1 if going up, 0 if going down
	
	reg [1:0] current_dir, next_dir;
	
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
			cur_x <= START_X;
			cur_y <= START_Y;
            next_x <= START_X;
            next_y <= START_Y;
			current_dir <= S_RIGHT;
            kill <= 1'b0;
		end
			
		else begin
			if (plot && ~kill) begin
					cur_x <= next_x;
					cur_y <= next_y;
					end
				/*
				if (redraw) begin
					
				end
				
				else begin
					cur_x <= next_x;
					cur_y <= next_y;
				end
				*/
			if (update) begin
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
			if ((next_y == TOP_EDGE && current_dir == S_UP) || (next_y == BOTTOM_EDGE && current_dir == S_DOWN)
			|| (next_x == RIGHT_EDGE && current_dir == S_RIGHT) || (next_x == LEFT_EDGE && current_dir == S_LEFT)) 
				kill <= 1'b1;
		end
	end
endmodule


module control(clk, resetn, go, plot, update, colour_out, kill);
	input clk;
	input resetn;
	input go;
    input kill;
	output reg [2:0] colour_out;

	output reg plot;
	output reg update;
//	output reg redraw;
	
	
	reg [2:0] colour;
	
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
	
	reg reset_frame;

	// wires for delay counter and frame counter
	wire [19:0] delay_ctr;
	wire [3:0] frame_ctr;

	reg [2:0] current_state, next_state;
	
	localparam		S_START				= 3'd0,
					S_DRAW		= 3'd1,
					S_DRAW_FINISH = 3'd2,
					S_WAIT				= 3'd3,
					S_UPDATE		=3'd4,
                    S_KILL          = 3'd5,
					BORDER_COLOUR 	= 3'b001,
					PLAYER_1_COLOUR = 3'b010,
					PLAYER_2_COLOUR = 3'b011;
					
					

	n_counter #(19) delay_counter(
		.clk(clk),
		.resetn(reset_frame),
		.d(5), // value should be 833334
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
			S_DRAW: next_state = S_DRAW_FINISH;
			S_DRAW_FINISH: next_state = S_WAIT;
			S_WAIT: next_state = ~|frame_ctr ? S_UPDATE : S_WAIT;
			S_UPDATE: next_state = kill ? S_KILL : S_DRAW;
            S_KILL: next_state = S_KILL;
			default: next_state = S_START;
		endcase
	end
	
	
	// Output logic
	always @(*) 
	begin: enable_signals
		plot = 1'b0;
		reset_frame = 1'b0;
		update = 1'b0;
//		redraw = 1'b0;
		
		case (current_state)
		/*
			S_START: begin
				plot = 1'b1;
				redraw = 1'b1;
			end
			*/
			
			S_DRAW: begin
				plot = 1'b1;
				colour_out = PLAYER_1_COLOUR;
			end

			S_DRAW_FINISH: begin
				plot = 1'b1;
				colour_out = PLAYER_1_COLOUR;
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
