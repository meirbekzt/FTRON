module n_counter
	(clk,
	resetn,
	d,
	enable,
	q
	);

	parameter N = 6;

	input clk;
	input resetn;
	input [N:0] d;
	input enable;
	output reg [N:0] q;

	always @(posedge clk) begin
		if (~resetn)
			q <= d;

		else if (enable) begin
			if (q == 0)
				q <= d;
			else 	
				q <= q - 1;
		end
	end
endmodule
