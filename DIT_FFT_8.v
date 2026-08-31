
module DIT_FFT_8(
 /* all signals in Q1.8 format */ 
 input wire  [8:0]	X_in0, X_in1, X_in2, X_in3, X_in4, X_in5, X_in6, X_in7, 							// real input signals (8 signals)
 output wire [8:0] 	Y_out0r, Y_out1r, Y_out2r, Y_out3r, Y_out4r, Y_out5r, Y_out6r, Y_out7r,		// complex output - real
 output wire [8:0] 	Y_out0i, Y_out1i, Y_out2i, Y_out3i, Y_out4i, Y_out5i, Y_out6i, Y_out7i		// complex output - imaginary
);

	/* all needed exponents values - real and imaginary */ 
 parameter W0_r = 9'b1; 			// W_8^0 real = 1
 parameter W0_i = 9'b0; 			// W_8^0 imag = 0
 
 parameter W1_r = 9'b010110101; 	// W_8^1 real = +0.707 ->  represent in Q1.8 -> scaling: (0.707 * 2^8) = +181  ->  0.10110101
 parameter W1_i = 9'b101001011; 	// W_8^1 imag = -0.707 ->  represent in Q1.8 -> 2's complement of(0.10110101)  ->  1.01001011
 
 parameter W2_r = 9'b0; 			// W_8^2 real = 0 
 parameter W2_i = 9'b111111111;		// W_8^2 imag = -1 (2's(1)) 
 
 parameter W3_r = 9'b101001011; 	// W_8^3 real = -0.707 ->  represent in Q1.8
 parameter W3_i = 9'b010110101;  	// W_8^3 imag = +0.707 ->  represent in Q1.8

 
	/* signal declarations */ 
 wire [8:0] Y3_r  [7:0]; 		// output of Stage-3 [2-point DFT's](real part)
 wire [8:0] Y3_i  [7:0]; 		// output of Stage-3 [2-point DFT's](imag part)
 wire [8:0] Y2_r  [7:0];		// output of stage-2 [4-point DFT's](real part)
 wire [8:0] Y2_i  [7:0];		// output of stage-2 [4-point DFT's](imag part)


			
			/* Butter-fly based structure for FFT */ 
		
	/* Stage 3 - Four butterflies for 4 2-point DFTs (most-left part) */ 
 bfly2_4 bf3_0(.x1(X_in0), .x2(X_in4), .W_r(W0_r), .W_i(W0_i), .Y1_r(Y3_r[0]), .Y1_i(Y3_i[0]), .Y2_r(Y3_r[1]), .Y2_i(Y3_i[1]));        	// 1st 2 point DFT
 bfly2_4 bf3_1(.x1(X_in2), .x2(X_in6), .W_r(W0_r), .W_i(W0_i), .Y1_r(Y3_r[2]), .Y1_i(Y3_i[2]), .Y2_r(Y3_r[3]), .Y2_i(Y3_i[3])); 		// 2nd 2 point DFT
 bfly2_4 bf3_2(.x1(X_in1), .x2(X_in5), .W_r(W0_r), .W_i(W0_i), .Y1_r(Y3_r[4]), .Y1_i(Y3_i[4]), .Y2_r(Y3_r[5]), .Y2_i(Y3_i[5])); 		// 3rd 2 point DFT
 bfly2_4 bf3_3(.x1(X_in3), .x2(X_in7), .W_r(W0_r), .W_i(W0_i), .Y1_r(Y3_r[6]), .Y1_i(Y3_i[6]), .Y2_r(Y3_r[7]), .Y2_i(Y3_i[7]));  		// 4th 2 point DFT
 
 
	/* Stage 2 - Four butterflies for 2 4-point DFTs (middle part) */
	// note that imaginary inputs (output from previous stage) are not involved as they're zeroes, 
	// because W0 = 1 and original inputs (time signals) are also real so previous imaginary outputs = 0.  
 bfly2_4 bf2_0(.x1(Y3_r[0]), .x2(Y3_r[2]), .W_r(W0_r), .W_i(W0_i), .Y1_r(Y2_r[0]), .Y1_i(Y2_i[0]), .Y2_r(Y2_r[2]), .Y2_i(Y2_i[2]));   	// 1st 4 point DFT
 bfly2_4 bf2_1(.x1(Y3_r[1]), .x2(Y3_r[3]), .W_r(W2_r), .W_i(W2_i), .Y1_r(Y2_r[1]), .Y1_i(Y2_i[1]), .Y2_r(Y2_r[3]), .Y2_i(Y2_i[3]));	
 bfly2_4 bf2_2(.x1(Y3_r[4]), .x2(Y3_r[6]), .W_r(W0_r), .W_i(W0_i), .Y1_r(Y2_r[4]), .Y1_i(Y2_i[4]), .Y2_r(Y2_r[6]), .Y2_i(Y2_i[6]));		// 2nd 4 point DFT 
 bfly2_4 bf2_3(.x1(Y3_r[5]), .x2(Y3_r[7]), .W_r(W2_r), .W_i(W2_i), .Y1_r(Y2_r[5]), .Y1_i(Y2_i[5]), .Y2_r(Y2_r[7]), .Y2_i(Y2_i[7]));
 
 
	/* Stage 1 - butterflies for 1 8-point DFT (most-right part) */ 
bfly2_4 bf1_0(.x1(Y2_r[0]), .x2(Y2_r[4]), .W_r(W0_r), .W_i(W0_i), .Y1_r(Y_out0r), .Y1_i(Y_out0i), .Y2_r(Y_out4r), .Y2_i(Y_out4i)); 
bfly2_4 bf1_2(.x1(Y2_r[2]), .x2(Y2_r[6]), .W_r(W2_r), .W_i(W2_i), .Y1_r(Y_out2r), .Y1_i(Y_out2i), .Y2_r(Y_out6r), .Y2_i(Y_out6i)); 

	// now we have here complex and fraction inputs, for first time 
bfly4_4 bf1_1(.x1_r(Y2_r[1]), .x1_i(Y2_i[1]), .x2_r(Y2_r[5]), .x2_i(Y2_i[5]), .W_r(W1_r), .W_i(W1_i), .Y1_r(Y_out1r), .Y1_i(Y_out1i), .Y2_r(Y_out5r), .Y2_i(Y_out5i)); 
bfly4_4 bf1_3(.x1_r(Y2_r[3]), .x1_i(Y2_i[3]), .x2_r(Y2_r[7]), .x2_i(Y2_i[7]), .W_r(W3_r), .W_i(W3_i), .Y1_r(Y_out3r), .Y1_i(Y_out3i), .Y2_r(Y_out7r), .Y2_i(Y_out7i)); 





endmodule



















