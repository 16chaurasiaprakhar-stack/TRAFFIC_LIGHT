module traffic_fsm #(parameter T_G1=10, T_G2=12, T_GC=10, T_AMB=3, T_CLR=2)(
  input  wire clk, rst_n,
  input  wire reqA, reqB, reqC, reqA_turn, reqB_turn, yield_en,
  output reg  A_R, A_Y, A_G1, A_G2, B_R, B_Y, B_G1, B_G2, C_R, C_Y, C_G,
  output reg  B_free_left, C_free_left);

  localparam [3:0] P0=0, P1=1, P2=2, P3=3, P4=4, P5=5, P6=6, P7=7;
  reg [3:0] state, next_state;
  reg [2:0] last;            // last served green phase (1..6)
  reg [7:0] cnt;             // down-counter, t_done when 0
  wire      t_done = (cnt == 0);
  wire [6:1] rq = {reqB_turn, reqA_turn, reqC, reqA & reqB, reqB & ~reqA, reqA & ~reqB};

  function [7:0] dur(input [3:0] s);
    case (s) P0: dur=T_CLR; P1,P2,P3: dur=T_G1; P4: dur=T_GC;
             P5,P6: dur=T_G2; P7: dur=T_AMB; default: dur=T_CLR; endcase
  endfunction

  function [2:0] pick(input [2:0] l, input [6:1] r);   // round-robin scan
    integer i; reg [3:0] c; reg [2:0] ls; reg found;
    begin
      ls = (l==0 || l>6) ? 3'd6 : l;
      found = 0; pick = (ls==6) ? 3'd1 : ls + 3'd1;      // fixed-order fallback
      for (i=1; i<=6; i=i+1) begin
        c = ls + i; if (c > 6) c = c - 6;
        if (!found && r[c]) begin pick = c[2:0]; found = 1; end
      end
    end
  endfunction

  // 1) state register
  always @(posedge clk or negedge rst_n)
    if (!rst_n) begin state<=P0; last<=3'd6; cnt<=T_CLR-1; end
    else begin
      state <= next_state;
      if (next_state != state) begin
        cnt <= dur(next_state) - 1;
        if (state==P0 && next_state>=P1 && next_state<=P6) last <= next_state[2:0];
      end else if (cnt != 0) cnt <= cnt - 1;
      if (last==0 || last>6) last <= 3'd6;
    end

  // 2) next-state logic (illegal codes fall to P0)
  always @* begin
    case (state)
      P0:                          next_state = t_done ? {1'b0, pick(last, rq)} : P0;
      P1,P2,P3,P4,P5,P6:           next_state = t_done ? P7 : state;
      P7:                          next_state = t_done ? P0 : P7;
      default:                     next_state = P0;
    endcase
  end

  // 3) Moore outputs
  always @* begin
    A_Y=0; A_G1=0; A_G2=0; B_Y=0; B_G1=0; B_G2=0; C_Y=0; C_G=0;
    B_free_left=0; C_free_left=0;
    case (state)
      P1: A_G1 = 1;
      P2: B_G1 = 1;
      P3: begin A_G1 = 1; B_G1 = 1; end
      P4: begin C_G = 1; C_free_left = yield_en; end
      P5: A_G2 = 1;
      P6: begin B_G2 = 1; B_free_left = yield_en; end
      P7: begin
            A_Y = (last==1) || (last==3) || (last==5);
            B_Y = (last==2) || (last==3) || (last==6);
            C_Y = (last==4);
          end
      default: ;                     // P0 and illegal codes: all red
    endcase
    A_R = ~(A_Y | A_G1 | A_G2);
    B_R = ~(B_Y | B_G1 | B_G2);
    C_R = ~(C_Y | C_G);
  end
endmodule
