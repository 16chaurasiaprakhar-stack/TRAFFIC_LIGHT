`timescale 1ns/1ps
// Self-checking testbench for traffic_fsm (checks V1-V10 from the FSM design PDF)
// Run:  iverilog -g2005 -o sim traffic_fsm.v tb_traffic_fsm.v && vvp sim && gtkwave tb_traffic_fsm.vcd
module tb_traffic_fsm;
  // short timers so the simulation is quick (each different, so dwell checks are meaningful)
  localparam TG1=3, TG2=4, TGC=5, TAMB=2, TCLR=2;

  reg clk = 0, rst_n = 0;
  reg reqA = 0, reqB = 0, reqC = 0, reqA_turn = 0, reqB_turn = 0, yield_en = 1;
  wire A_R, A_Y, A_G1, A_G2, B_R, B_Y, B_G1, B_G2, C_R, C_Y, C_G;
  wire B_free_left, C_free_left;

  traffic_fsm #(.T_G1(TG1), .T_G2(TG2), .T_GC(TGC), .T_AMB(TAMB), .T_CLR(TCLR)) dut (
    .clk(clk), .rst_n(rst_n),
    .reqA(reqA), .reqB(reqB), .reqC(reqC), .reqA_turn(reqA_turn), .reqB_turn(reqB_turn),
    .yield_en(yield_en),
    .A_R(A_R), .A_Y(A_Y), .A_G1(A_G1), .A_G2(A_G2),
    .B_R(B_R), .B_Y(B_Y), .B_G1(B_G1), .B_G2(B_G2),
    .C_R(C_R), .C_Y(C_Y), .C_G(C_G),
    .B_free_left(B_free_left), .C_free_left(C_free_left));

  always #5 clk = ~clk;

  integer errors = 0;
  reg     chk_en = 0;

  task fail(input [8*48-1:0] msg);
    begin
      errors = errors + 1;
      $display("[FAIL] t=%0t state=%0d : %0s", $time, dut.state, msg);
    end
  endtask

  // ---------------------------------------------------------------
  // V1 / V5-V9 : lamp-level safety checks, sampled just before each clock edge
  // ---------------------------------------------------------------
  wire legal =
      (A_R  & B_R  & C_R ) |                       // P0
      (A_G1 & B_R  & C_R ) | (A_R & B_G1 & C_R) |  // P1, P2
      (A_G1 & B_G1 & C_R ) | (A_R & B_R  & C_G) |  // P3, P4
      (A_G2 & B_R  & C_R ) | (A_R & B_G2 & C_R) |  // P5, P6
      (A_Y  & B_R  & C_R ) | (A_R & B_Y  & C_R) |  // amber rows T1, T2
      (A_Y  & B_Y  & C_R ) | (A_R & B_R  & C_Y);   // amber rows T3, T4

  always @(posedge clk) if (chk_en) begin
    if (legal !== 1'b1)                                   fail("V1 illegal lamp combination");
    if ((A_R+A_Y+A_G1+A_G2) !== 1)                        fail("V8 signal A not one-hot");
    if ((B_R+B_Y+B_G1+B_G2) !== 1)                        fail("V8 signal B not one-hot");
    if ((C_R+C_Y+C_G) !== 1)                              fail("V8 signal C not one-hot");
    if (A_G1 && B_G1 && dut.state !== 4'd3)               fail("V5 A_G1+B_G1 outside P3");
    if (C_G && !(A_R && B_R))                             fail("V6 C_G overlaps A/B");
    if (A_G2 && !(B_R && C_R))                            fail("V7 A_G2 not exclusive");
    if (B_G2 && !(A_R && C_R))                            fail("V7 B_G2 not exclusive");
    if (dut.state === 4'd0 && !(A_R && B_R && C_R))       fail("V9 P0 not all red");
    if (dut.state === 4'd7) begin                          // amber mirrors the last green phase
      if (A_Y !== (dut.last==1 || dut.last==3 || dut.last==5)) fail("P7 A amber wrong");
      if (B_Y !== (dut.last==2 || dut.last==3 || dut.last==6)) fail("P7 B amber wrong");
      if (C_Y !== (dut.last==4))                               fail("P7 C amber wrong");
    end
    if (B_free_left !== (dut.state==4'd6 && yield_en))    fail("B_free_left wrong");
    if (C_free_left !== (dut.state==4'd4 && yield_en))    fail("C_free_left wrong");
  end

  // ---------------------------------------------------------------
  // V2 : sequence monitor (Pk -> P7 -> P0 -> Pj) and dwell-time check
  // ---------------------------------------------------------------
  reg [3:0]  prev = 0, cur;
  integer    dwell = 0;
  reg        skip = 1;              // skip checks on the next state change (after reset / forced state)
  reg        ok;
  integer    green_count = 0;
  reg [3:0]  last_green = 0;
  integer    served [1:6];
  reg [7:0]  seen = 0;
  integer    k;
  initial for (k=1; k<=6; k=k+1) served[k] = 0;

  function integer exp_dwell(input [3:0] s);
    begin
      case (s)
        4'd0: exp_dwell = TCLR;
        4'd1, 4'd2, 4'd3: exp_dwell = TG1;
        4'd4: exp_dwell = TGC;
        4'd5, 4'd6: exp_dwell = TG2;
        4'd7: exp_dwell = TAMB;
        default: exp_dwell = -1;
      endcase
    end
  endfunction

  always @(negedge clk) if (chk_en) begin
    cur = dut.state;
    if (cur < 8) seen[cur] = 1'b1;
    if (!rst_n) begin
      skip = 1; prev = cur; dwell = 0;
    end else if (cur === prev) begin
      dwell = dwell + 1;
    end else begin
      if (!skip) begin
        if      (prev == 4'd0) ok = (cur >= 1 && cur <= 6);
        else if (prev == 4'd7) ok = (cur == 4'd0);
        else if (prev <= 4'd6) ok = (cur == 4'd7);
        else                   ok = (cur == 4'd0);      // recovery from an illegal code
        if (!ok) fail("V2 illegal state transition");
        if (prev <= 4'd7 && dwell != exp_dwell(prev)) begin
          $display("[FAIL] t=%0t dwell in P%0d was %0d, expected %0d", $time, prev, dwell, exp_dwell(prev));
          errors = errors + 1;
        end
      end
      skip = 0; dwell = 1; prev = cur;
      if (cur >= 1 && cur <= 6) begin
        green_count = green_count + 1; last_green = cur; served[cur] = served[cur] + 1;
      end
    end
  end

  // ---------------------------------------------------------------
  // Directed-test helpers
  // ---------------------------------------------------------------
  task set_req(input a, input b, input c, input at, input bt);
    begin
      @(negedge clk);
      reqA = a; reqB = b; reqC = c; reqA_turn = at; reqB_turn = bt;
    end
  endtask

  // wait for the next green phase to start and compare with the expected one
  task expect_next(input [3:0] expected);
    integer c0, n;
    begin
      c0 = green_count; n = 0;
      while (green_count == c0 && n < 200) begin @(negedge clk); #1; n = n + 1; end
      if (n >= 200)                 fail("timeout waiting for next green");
      else if (last_green !== expected) begin
        $display("[FAIL] t=%0t next green was P%0d, expected P%0d", $time, last_green, expected);
        errors = errors + 1;
      end else
        $display("[ok]   next green P%0d as expected", last_green);
    end
  endtask

  // V3 : reset from the middle of a phase
  task test_reset;
    begin
      @(negedge clk); skip = 1; rst_n = 0; #1;
      if (dut.state !== 4'd0)             fail("V3 reset did not force P0");
      if (!(A_R && B_R && C_R))           fail("V3 lamps not red in reset");
      repeat (2) @(negedge clk);
      rst_n = 1;
      $display("[ok]   reset test done");
    end
  endtask

  // V4 : force each unused state code, expect all-red and recovery to P0
  task test_illegal;
    integer j;
    begin
      for (j = 8; j < 16; j = j + 1) begin
        @(negedge clk); skip = 1;
        force dut.state = j[3:0]; #1;
        if (!(A_R && B_R && C_R))         fail("V4 illegal state lamps not red");
        release dut.state;
        @(posedge clk); #1;
        if (dut.state !== 4'd0)           fail("V4 no recovery to P0");
      end
      $display("[ok]   illegal-state recovery done");
    end
  endtask

  // ---------------------------------------------------------------
  // Main sequence
  // ---------------------------------------------------------------
  integer i;
  initial begin
    $dumpfile("tb_traffic_fsm.vcd");
    $dumpvars(0, tb_traffic_fsm);

    reqC = 1;                               // only C is requesting during the first cycle
    repeat (2) @(posedge clk);
    @(negedge clk); chk_en = 1; prev = dut.state;
    @(negedge clk); rst_n = 1;

    // 1) demand selection (round-robin starts after P6, so only rq4 -> P4)
    expect_next(4);

    // 2) fixed-order fallback: no requests -> P5,P6,P1,P2,P3,P4
    set_req(0,0,0,0,0);
    expect_next(5); expect_next(6); expect_next(1);
    expect_next(2); expect_next(3); expect_next(4);

    // 3) every phase selected by its own request
    set_req(1,0,0,0,0); expect_next(1);     // A only        -> P1
    set_req(0,1,0,0,0); expect_next(2);     // B only        -> P2
    set_req(1,1,0,0,0); expect_next(3);     // A and B       -> P3
    set_req(0,0,1,0,0); expect_next(4);     // C             -> P4
    set_req(0,0,0,1,0); expect_next(5);     // A turn/U      -> P5
    set_req(0,0,0,0,1); expect_next(6);     // B turn/U      -> P6

    // 4) round-robin fairness: A-only and C both request, last served was P6 -> P1 then P4
    set_req(1,0,1,0,0); expect_next(1); expect_next(4); expect_next(1);

    // 5) reset in mid-phase, then illegal-state recovery
    set_req(0,0,0,0,0);
    expect_next(2);                         // last served was P1 -> fixed order gives P2
    test_reset;
    expect_next(1);                         // reset sets last=P6, no requests -> fixed order starts at P1
    test_illegal;

    // 6) long random stimulus
    for (i = 0; i < 4000; i = i + 1) begin
      @(negedge clk);
      reqA = $random; reqB = $random; reqC = $random;
      reqA_turn = $random; reqB_turn = $random;
      if (i % 97 == 0) yield_en = $random;
    end
    repeat (30) @(negedge clk);

    // 7) coverage summary
    if (seen !== 8'hFF) fail("not all states P0..P7 visited");
    $display("served count P1..P6 = %0d %0d %0d %0d %0d %0d",
             served[1], served[2], served[3], served[4], served[5], served[6]);
    for (k = 1; k <= 6; k = k + 1) if (served[k] == 0) fail("a phase was never served");

    if (errors == 0) $display("*** ALL TESTS PASSED ***");
    else             $display("*** %0d ERROR(S) ***", errors);
    $finish;
  end

  initial begin #5000000; $display("[FAIL] global timeout"); $finish; end
endmodule
