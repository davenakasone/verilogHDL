/*
	the GCD algorithm, given 2 numbers,   
        the largest number that can divide both with remainder 0

    unsinged reg[7:0] A;
    unsinged reg[7:0] B;

    while (A !=  B)
    {
        if (A > B)
        {
            A = A - B;
        }
        else
        {
            B = B - A;
        }
    }
    return A;
*/

`timescale 1ns/1ns 
`define HALF 5
`define START 0   
`define STOP 255
`define SEED 3174
`define DEBUG_TB -999
`define DEBUG_WAVE -888
//`define DEBUG_CU -777
//`define DEBUG_DP -666


module tb();
	wire [7:0] result;
	wire done;
	reg [7:0] A_input;
    reg [7:0] B_input;
	reg start;
    reg reset;
    reg clock;

	integer idx;
    integer seed;

    general_data_path GDP (
                            .result(result),
                            .done(done),
                            .A_input(A_input),
                            .B_input(B_input),
                            .start(start), 
                            .reset(reset), 
                            .clock(clock)
                        );

	always begin
		#`HALF;
		clock = ~clock;
	end
	
	initial begin
        `ifdef DEBUG_WAVE
		    $dumpfile("wave.vcd");
            $dumpvars(0, tb);
        `endif
		$write("\n testing the GCD algorithm, %d random A/B pairs\n\n", `STOP+1);
        $write("    input  \"A\"      |    input \"B\"       |            GCD        |  expected\n");
        $write(" ----------------------------------------------------------------------------\n");
        seed = `SEED;
		start = 1'b1;
		reset = 1'b0;
		clock = 1'b0;

        for (idx = `START; idx <= `STOP; idx = idx + 1) begin
            reset = 1'b0;
            //A_input = $random(seed) % 255;
            B_input = $random(seed) % 255;
            //B_input = 8'b0000_1001;
            //A_input = 8'b0000_0011;
            A_input = idx;
            //B_input = idx-1;

            while (done != 1'b1) begin
                #(`HALF + `HALF);
            end

            `ifdef DEBUG_TB
                $write("  {%3d}  %4b_%4b  |  {%3d}  %4b_%4b  |  %4b_%4b -->   %3d  |  ",
                    A_input, A_input[7:4], A_input[3:0], 
                    B_input, B_input[7:4], B_input[3:0], 
                    result[7:4], result[3:0], result);
                getGCD(A_input, B_input, result);
            `endif
            
            reset = 1'b1;
            #(`HALF + `HALF);
        end

        #3;
        $write("\n\n\t\t ~ ~ ~ TEST COMPLETE ~ ~ ~  %7t  ns\n\n", $time);
		$finish;
	end

    task automatic getGCD;
        input reg [7:0] aReg;
        input reg [7:0] bReg;
        input reg [7:0] from_GDP;
        integer aa;
        integer bb;
        integer check;

        begin
            aa = aReg;
            bb = bReg;
            check = from_GDP;
            if (aa != 0 && bb != 0) begin
                while (aa != bb) begin
                    if (aa > bb) begin
                        aa = aa - bb;
                    end
                    else begin
                        bb = bb - aa;
                    end
                end
            end
            if (check == aa) begin
                $write("%3d        CORRECT\n", aa);
            end
            else begin
                $write("%3d        !!!WRONG!!!\n", aa);
            end
        end
    endtask

endmodule


////~~~~ top entity, combines control unit and data path
module general_data_path (
                            output wire [7:0] result,
                            output wire done,
                            input wire [7:0] A_input,
                            input wire [7:0] B_input,
                            input wire start, 
                            input wire reset, 
                            input wire clock
                        );	
	wire [1:0] IE;
    wire WE;
    wire [1:0] WA;
    wire RAE;
    wire [1:0] RAA;
    wire RBE;
    wire [1:0] RBA;
    wire [2:0] ALU;
    wire [1:0] SH;
    wire OE;
    wire n_is_0; 
    wire A_gt_B;
    wire A_is_B;

    assign done = OE;

    control_unit CU (
                        .IE(IE), 
                        .WE(WE),
                        .WA(WA),
                        .RAE(RAE),
                        .RAA(RAA),
                        .RBE(RBE),
                        .RBA(RBA),
                        .ALU(ALU),
                        .SH(SH),
                        .OE(OE),
                        .n_is_0(n_is_0),
                        .A_gt_B(A_gt_B),
                        .A_is_B(A_is_B),
                        .start(start),
                        .reset(reset),
                        .clock(clock)
                    );

    data_path DP (
                    .n_is_0(n_is_0),
                    .A_gt_B(A_gt_B),
                    .A_is_B(A_is_B),
                    .result(result),
                    .A_input(A_input),
                    .B_input(B_input),
                    .IE(IE),
                    .WE(WE),
                    .WA(WA),
                    .RAE(RAE),
                    .RAA(RAA),
                    .RBE(RBE),
                    .RBA(RBA),
                    .ALU(ALU),
                    .SH(SH),
                    .OE(OE),
                    .clock(clock)
                );
endmodule


////~~~~ control unit applied to data path
module control_unit (
                        output wire [1:0] IE, 
                        output wire WE,
                        output wire [1:0] WA,
                        output wire RAE,
                        output wire [1:0] RAA,
                        output wire RBE,
                        output wire [1:0] RBA,
                        output wire [2:0] ALU,
                        output wire [1:0] SH,
                        output wire OE,
                        input wire n_is_0,
                        input wire A_gt_B,
                        input wire A_is_B,
                        input wire start,
                        input wire reset,
                        input wire clock
                    );

	reg [2:0] state;
	reg [2:0] n_state;

	parameter s0 = 3'b000;
	parameter s1 = 3'b001;
	parameter s2 = 3'b010;
	parameter s3 = 3'b011;
	parameter s4 = 3'b100;
    parameter s5 = 3'b101;
    parameter s6 = 3'b110;
	
	initial begin
        state = s0;
    end
	
	always @ (posedge clock) begin
        if (reset == 1'b1) n_state <= s0;  // interupt all
        `ifdef DEBUG_CU
            $write("\n st   nst  || IE | WE | WA | RAE | RAA | RBE | RBA | ALU | SH | OE || start | reset | n==0  | A>B  |   A == B     %7t ns\n",
                $time);
            $write("-------------------------------------------------------------------------------------------\n");
            $write(" %3b  %3b  || %1b  | %1b  | %2b |  %1b  | %2b  |  %1b  | %2b  | %3b | %2b | %1b  ||   %1b   |   %1b   |  %1b  |  %1b  |  %1b\n", 
                    state, n_state, IE,    WE,    WA,  RAE,    RAA,    RBE,   RBA,   ALU,   SH,   OE,   start, reset, n_is_0, A_gt_B, A_is_B);
        `endif
        state <= n_state;
    end

    always @ (*) begin
        case(state)
            s0 :                                          // rf[00] <-- 0  make zero  ,  wait for start
                begin
                    n_state = (start) ? s1 : s0; 
                end           
            s1 :                                          // rf[00] <-- A ,  get input
                begin
                    n_state = (n_is_0) ? s6 : s2;
                end
            s2 :                                          // rf[01] <-- B ,  get input
                begin
                    n_state = (n_is_0) ? s6 : s3;
                end                          
            s3 :                                          // rf[00] - rf[01] , no write "A-B"
                begin
                    if (A_is_B) begin
                        n_state = s6;
                    end
                    else begin
                        n_state = (A_gt_B) ? s4 : s5;
                    end
                end
            s4 :                                          // rf[00] <-- rf [00] - rf[01] , A > B
                begin
                    n_state = s3; 
                end                           
            s5 :                                          // rf[01] <-- rf [01] - rf[00] , ~(A > B)
                begin
                    n_state = s3; 
                end                               
            s6 :                                          // show rf[00]  ,  output and wait for reset
                begin
                    n_state = (reset) ? s0 : s6;
                end              
            default :                       // stop the latch
                begin
                    n_state = s0; 
                end
        endcase
	end

	assign IE[0] = (state == s1);
    assign IE[1] = (state == s2); 
		
	assign WE = (state == s1) || 
                (state == s2) || 
                (state == s4) || 
                (state == s5);
                
	assign WA[1] = 1'b0;
	assign WA[0] = (state == s2) || 
                   (state == s5);
		
	assign RAE = (state == s1) || 
                 (state == s2) || 
                 (state == s3) || 
                 (state == s4) || 
                 (state == s5) || 
                 (state == s6);
	assign RAA[1] = 1'b0;
	assign RAA[0] = (state == s2) || 
                    (state == s5);
		
	assign RBE = (state == s3) ||
                 (state == s4) ||
                 (state == s5);
	assign RBA[1] = 1'b0;
	assign RBA[0] = (state == s3) ||
                    (state == s4);
		
	assign ALU[2] = (state == s4) || 
                    (state == s5);
	assign ALU[1] = 1'b0;
	assign ALU[0] = (state == s4) ||  
                    (state == s5);
		
	assign SH[1] = 1'b0;
	assign SH[0] = 1'b0;
	assign OE = (state == s6);	
endmodule


////~~~~ data path combining the 5 components 
module data_path (
                    output wire A_gt_B,
                    output wire A_is_B,
                    output wire n_is_0,
                    output wire [7:0] result,
                    input wire [7:0] A_input,
                    input wire [7:0] B_input,
                    input wire [1:0] IE,
                    input wire WE,
                    input wire [1:0] WA,
                    input wire RAE,
                    input wire [1:0] RAA,
                    input wire RBE,
                    input wire [1:0] RBA,
                    input wire [2:0] ALU,
                    input wire [1:0] SH,
                    input wire OE,
                    input wire clock
                );

    parameter BLANK = 8'b0000_0000;  
    wire [7:0] from_mux;          
	reg [7:0] to_register_file;
    wire [7:0] rf_port_A;
    wire [7:0] rf_port_B;
    wire [7:0] from_alu;
    wire [7:0] from_shifter;
	
    /*
        This internal register completes the feedback.
        Unlike the other connections, it must be a register.
        It is very important and provides the initialization,
        along with proper transfer sequence.
        To use a wire, all subordinate modules would need to be structural.
    */
    initial begin
        to_register_file = BLANK;
    end
    always @ (*) begin
        to_register_file = from_mux;
    end
	
    /*
        This is the probe right under the MUX.
        It won't fail if you move states correctly.
        Don't get too crazy with the logic checks.
    */
	assign n_is_0 = (from_mux == 8'b0000_0000); 
    
    compare AB (
                    .A_gt_B(A_gt_B),
                    .A_is_B(A_is_B),
                    .alu_A(rf_port_A),
                    .alu_B(rf_port_B)
                );

    c1_mux comp1 (
                    .from_mux(from_mux),
                    .A_input(A_input), 
                    .B_input(B_input), 
                    .feedback(from_shifter),
                    .IE(IE)
                );

    c2_register_file comp2 (
                            .port_A(rf_port_A),
                            .port_B(rf_port_B),     
                            .from_mux(to_register_file),
                            .WE(WE),                
                            .WA(WA),          
                            .RAE(RAE),    
                            .RAA(RAA),         
                            .RBE(RBE),            
                            .RBA(RBA),      
                            .clock(clock)
                        ); 

    c3_alu comp3 (
                    .from_alu(from_alu),
                    .A_bus(rf_port_A),
                    .B_bus(rf_port_B),
                    .ALU(ALU)
                );

    c4_shifter comp4 (
                        .from_shifter(from_shifter),
                        .from_alu(from_alu),
                        .SH(SH)
                    );

    c5_triBuff comp5 (
                        .result(result), 
                        .from_shifter(from_shifter), 
                        .OE(OE)
                    );
    
    `ifdef DEBUG_DP
        always @ (posedge clock) begin
            #1;
            $write("\n ************************************************\n");
            $write(" {data_path}                               %7t ns\n", $time);
            $write("               n_is_0:  %1b\n", n_is_0);
            $write(" -------------------------\n");
            $write("                   IE:  %2b\n", IE);
            $write("  mux[00], shifter_out:  %4b_%4b\n", from_shifter[7:4], from_shifter[3:0]);
            $write("         mux[01], A_in:  %4b_%4b\n", A_input[7:4], A_input[3:0]);
            $write("         mux[10], B_in:  %4b_%4b\n", B_input[7:4], B_input[3:0]);
            $write("          mux out, nnn:  %4b_%4b\n", from_mux[7:4], from_mux[3:0]);
            $write(" ---------------------------------\n");
            $write("               rf_inp:  %4b_%4b\n", to_register_file[7:4], to_register_file[3:0]);
            $write(" WE:   %1b  ,  WA:   %2b\n", WE, WA);
            $write(" RAE:  %1b  ,  RAA:  %2b\n", RAE, RAA);
            $write(" RBE:  %1b  ,  RBA:  %2b\n", RBE, RBA);
            $write(" reg[00]:  %4b_%4b \"A\"\n", comp2.internal[2'b00][7:4], comp2.internal[2'b00][3:0]);
            $write(" reg[01]:  %4b_%4b \"B\"\n"  , comp2.internal[2'b01][7:4], comp2.internal[2'b01][3:0]);
            $write(" reg[10]:  %4b_%4b\n"        , comp2.internal[2'b10][7:4], comp2.internal[2'b10][3:0]);
            $write(" reg[11]:  %4b_%4b\n"        , comp2.internal[2'b11][7:4], comp2.internal[2'b11][3:0]);
            $write("               port A:  %4b_%4b\n", rf_port_A[7:4], rf_port_A[3:0]);
            $write("               port B:  %4b_%4b\n", rf_port_B[7:4], rf_port_B[3:0]);
            $write(" --------------------------------------\n");
            $write("        alu operation:  %3b\n", ALU);
            $write("           alu output:  %4b_%4b\n", from_alu[7:4], from_alu[3:0]);
            $write(" ---------------------------------\n");
            $write("    shifter operation:  %3b\n", SH);
            $write("       shifter output:  %4b_%4b\n", from_shifter[7:4], from_shifter[3:0]);
            $write(" ---------------------------------\n");
            $write("                   OE:  %1b\n", OE);
            $write("               result:  %4b_%4b", result[7:4], result[3:0]);
            if (OE == 1'b1) $write("   !!!SUCCESS!!!\n");
            else $write("      ...working\n");
            $write(" ************************************************\n");
        end
    `endif
endmodule


////~~~~ 2-to-1 mux, 8 bits, component 1 of 5
module c1_mux (
                    output reg [7:0] from_mux,
                    input wire [7:0] A_input, 
                    input wire [7:0] B_input, 
                    input wire [7:0] feedback,
                    input wire [1:0] IE
                );
	
	always @ (*) begin
        case(IE)
            2'b00 : from_mux = feedback;
            2'b01 : from_mux = A_input;
            2'b10 : from_mux = B_input;
            default : from_mux = feedback; 
        endcase
    end
endmodule


////~~~~ register file, component 2 of 5
module c2_register_file(
                            output wire [7:0] port_A,     // value assigned to port A
                            output wire [7:0] port_B,     // value assigned to port B
                            input wire [7:0] from_mux,    // bus after MUX selection
                            input wire WE,                // write enable
                            input wire [1:0] WA,          // write address
                            input wire RAE,               // port A read enable
                            input wire [1:0] RAA,         // port A Read address 
                            input wire RBE,               // port B read enable
                            input wire [1:0] RBA,         // port B read address
                            input wire clock
                        ); 
	
    parameter BLANK = 8'b0000_0000;

    // x4 8-bit registers "4x8", index them in binary
	reg [7:0] internal [3:0]; 

    // only write if WE is active
	always @ (posedge clock) begin
        if (WE == 1'b1) internal[WA] <= from_mux;
    end
     
    // reading to ports is done in combination, if enabled
    assign port_A = (RAE == 1'b1) ? internal[RAA] : BLANK; 
    assign port_B = (RBE == 1'b1) ? internal[RBA] : BLANK;
endmodule


////~~~~ ALU, component 3 of 5
module c3_alu (
                output reg [7:0] from_alu,
                input wire [7:0] A_bus,
                input wire [7:0] B_bus,
                input wire [2:0] ALU
            );

    parameter pass_A = 3'b000;     // bus A passes through
    parameter A_and_B = 3'b001;    // bitwise AND, A & B
    parameter A_or_B = 3'b010;     // bitwise OR, A | B
    parameter not_A = 3'b011;      // bitwise negate, ~A
    parameter A_add_B = 3'b100;    // addition, A + B
    parameter A_sub_B = 3'b101;    // subtraction, A - B
    parameter inc_A = 3'b110;      // increment, A = A + 1
    parameter dec_A = 3'b111;      // decrement, A = A - 1

    always @ (*) begin 
        case(ALU) 
            pass_A  : from_alu = A_bus;                  
            A_and_B : from_alu = A_bus & B_bus;                  
            A_or_B  : from_alu = A_bus | B_bus; 
            not_A   : from_alu = ~A_bus;
            A_add_B : from_alu = A_bus + B_bus;
            A_sub_B : from_alu = A_bus - B_bus;
            inc_A   : from_alu = A_bus + 1'b1;
            dec_A   : from_alu = A_bus - 1'b1;                
        endcase
    end
endmodule


////~~~~ shifter, componenet 4 of 5
module c4_shifter (
                    output reg [7:0] from_shifter,
                    input wire [7:0] from_alu,
                    input wire [1:0] SH
                );

    parameter PASS = 2'b00;      // pass through
    parameter LEFT = 2'b01;      // left shift and fill with 0's
    parameter RIGHT = 2'b10;     // right shift and fill with 0's
    parameter ROTATE = 2'b11;    // rotate right

    always @ (*) begin 
        case(SH) 
            PASS   : from_shifter = from_alu;                  
            LEFT   : from_shifter = from_alu << 1;                 
            RIGHT  : from_shifter = from_alu >> 1;
            ROTATE : from_shifter = {from_alu[6:0], from_alu[7]};                    
        endcase
    end
endmodule


////~~~~ final output buffer, componenet 5 of 5
module c5_triBuff (
                    output reg [7:0] result, 
                    input wire [7:0] from_shifter, 
                    input wire OE
                );

	always @ (*) begin
        result = (OE == 1'b1) ? from_shifter : 8'bzzzz_zzzz;
    end		
endmodule


////~~~~ a comparator to probe ALU input lines for A > B, A==B
module compare (
                    output wire A_gt_B,
                    output wire A_is_B,
                    input wire [7:0] alu_A,
                    input wire [7:0] alu_B
                );
    assign A_gt_B = (alu_A > alu_B) ? 1'b1 : 1'b0;
    assign A_is_B = (alu_A == alu_B) ? 1'b1 : 1'b0;
endmodule


////////~~~~~~~~END>  gcd.v
