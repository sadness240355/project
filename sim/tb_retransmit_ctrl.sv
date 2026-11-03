`timescale 1ns/1ps
module tb;
localparam C_QP_INDX_WIDTH=5, C_WQE_FIFO_IDX_WIDTH=2;
reg  core_clk = 0;
reg  core_rstn = 0;
reg  i_retransmit_reqd = 0;
reg [C_QP_INDX_WIDTH -1 : 0] i_retransmit_qpid = 0;
reg [                55 : 0] i_retransmit_wqe_bram_din = 0;
wire  o_retransmit_accepted;
reg [                  23:0] i_psn_to_retry = 0;
reg [                  23:0] i_ssn_to_retry = 0;
reg [C_WQE_FIFO_IDX_WIDTH : 0] i_num_valid_entries = 0;
wire  o_qp_cq_hdptr_req;
reg  i_qp_cq_hdptr_valid = 0;
reg [                15:0] i_qp_cq_hdptr = 0;
wire [C_QP_INDX_WIDTH -1:0] o_qp_cq_hdptr_idx;
wire [                15:0] o_qp_curr_sqptr_proc;
wire  o_qp_curr_sqptr_proc_wen;
wire [C_QP_INDX_WIDTH -1:0] o_qp_curr_sqptr_proc_idx;
wire [                23:0] o_qp_stat_ret_sq_psn;
wire  o_qp_stat_ret_sq_psn_wen;
wire [C_QP_INDX_WIDTH -1:0] o_qp_stat_ret_sq_psn_idx;
reg  i_qp_stat_ret_sq_psn_valid = 0;
wire  o_qp_sq_psn_req;
reg  i_qp_sq_psn_valid = 0;
reg [                23:0] i_qp_sq_psn = 0;
wire  o_qp_sq_psn_wrn;
wire [                23:0] o_qp_sq_psn;
wire [C_QP_INDX_WIDTH -1:0] o_qp_sq_psn_idx;
wire [C_QP_INDX_WIDTH -1:0] o_qp_stat_ssn_idx;
wire [                23:0] o_qp_stat_ssn;
reg [                23:0] i_qp_stat_ssn = 0;
wire  o_qp_stat_ssn_req;
wire  o_qp_stat_ssn_wrn;
reg  i_qp_stat_ssn_valid = 0;
reg [511:0] i_wqe = 0;
wire [511:0] o_wqe;
wire  o_wqe_fifo_push;
wire  o_wqe_fifo_pop;
wire  o_halt;
wire [C_QP_INDX_WIDTH -1 : 0] o_halted_qpid;
reg  i_halted = 0;
reg  i_osq_pop = 0;
reg [C_QP_INDX_WIDTH-1 : 0] i_osq_pop_qpid = 0;
wire  o_wqe_dropped;
wire [C_QP_INDX_WIDTH-1 : 0] o_wqe_dropped_qpid;
wire [                 56:0] o_retransmit_wqe_bram_din;
reg [                 56:0] i_retransmit_wqe_bram_dout = 0;
wire  o_retransmit_wqe_bram_wen;
wire [C_QP_INDX_WIDTH-1 : 0] o_retransmit_wqe_bram_idx;
reg  i_prog_full_db_update_curr_sqptr = 0;
lrdma_qp_retransmit_ctrl #(.C_NUM_QP(32),.C_QP_INDX_WIDTH(5),.C_WQE_FIFO_IDX_WIDTH(2)) dut(
.core_clk(core_clk),
.core_rstn(core_rstn),
.i_retransmit_reqd(i_retransmit_reqd),
.i_retransmit_qpid(i_retransmit_qpid),
.i_retransmit_wqe_bram_din(i_retransmit_wqe_bram_din),
.o_retransmit_accepted(o_retransmit_accepted),
.i_psn_to_retry(i_psn_to_retry),
.i_ssn_to_retry(i_ssn_to_retry),
.i_num_valid_entries(i_num_valid_entries),
.o_qp_cq_hdptr_req(o_qp_cq_hdptr_req),
.i_qp_cq_hdptr_valid(i_qp_cq_hdptr_valid),
.i_qp_cq_hdptr(i_qp_cq_hdptr),
.o_qp_cq_hdptr_idx(o_qp_cq_hdptr_idx),
.o_qp_curr_sqptr_proc(o_qp_curr_sqptr_proc),
.o_qp_curr_sqptr_proc_wen(o_qp_curr_sqptr_proc_wen),
.o_qp_curr_sqptr_proc_idx(o_qp_curr_sqptr_proc_idx),
.o_qp_stat_ret_sq_psn(o_qp_stat_ret_sq_psn),
.o_qp_stat_ret_sq_psn_wen(o_qp_stat_ret_sq_psn_wen),
.o_qp_stat_ret_sq_psn_idx(o_qp_stat_ret_sq_psn_idx),
.i_qp_stat_ret_sq_psn_valid(i_qp_stat_ret_sq_psn_valid),
.o_qp_sq_psn_req(o_qp_sq_psn_req),
.i_qp_sq_psn_valid(i_qp_sq_psn_valid),
.i_qp_sq_psn(i_qp_sq_psn),
.o_qp_sq_psn_wrn(o_qp_sq_psn_wrn),
.o_qp_sq_psn(o_qp_sq_psn),
.o_qp_sq_psn_idx(o_qp_sq_psn_idx),
.o_qp_stat_ssn_idx(o_qp_stat_ssn_idx),
.o_qp_stat_ssn(o_qp_stat_ssn),
.i_qp_stat_ssn(i_qp_stat_ssn),
.o_qp_stat_ssn_req(o_qp_stat_ssn_req),
.o_qp_stat_ssn_wrn(o_qp_stat_ssn_wrn),
.i_qp_stat_ssn_valid(i_qp_stat_ssn_valid),
.i_wqe(i_wqe),
.o_wqe(o_wqe),
.o_wqe_fifo_push(o_wqe_fifo_push),
.o_wqe_fifo_pop(o_wqe_fifo_pop),
.o_halt(o_halt),
.o_halted_qpid(o_halted_qpid),
.i_halted(i_halted),
.i_osq_pop(i_osq_pop),
.i_osq_pop_qpid(i_osq_pop_qpid),
.o_wqe_dropped(o_wqe_dropped),
.o_wqe_dropped_qpid(o_wqe_dropped_qpid),
.o_retransmit_wqe_bram_din(o_retransmit_wqe_bram_din),
.i_retransmit_wqe_bram_dout(i_retransmit_wqe_bram_dout),
.o_retransmit_wqe_bram_wen(o_retransmit_wqe_bram_wen),
.o_retransmit_wqe_bram_idx(o_retransmit_wqe_bram_idx),
.i_prog_full_db_update_curr_sqptr(i_prog_full_db_update_curr_sqptr)
);

always #2 core_clk=~core_clk;
integer cycle=0,fd,pops=0,pushes=0,drops=0,commits=0,cases=0;
reg [23:0] expected_psn,expected_ssn;
reg [55:0] expected_payload;
reg expected_toggle;
reg [4:0] expected_qp;
always @(posedge core_clk) begin
 if(core_rstn) begin
  cycle=cycle+1;
  $fdisplay(fd,"%0d,%0d,%b,%b,%b,%b,%b,%h",cycle,dut.retry_cs,o_halt,o_wqe_fifo_pop,o_wqe_fifo_push,o_wqe_dropped,o_retransmit_wqe_bram_wen,o_qp_sq_psn);
  if(o_qp_sq_psn_wrn) begin
   if(o_qp_sq_psn!==expected_psn || o_qp_sq_psn_idx!==expected_qp) $fatal(1,"PSN rollback mismatch");
   if(o_qp_stat_ssn!==expected_ssn || o_qp_curr_sqptr_proc!==16'h2a) $fatal(1,"context rollback mismatch");
  end
  if(o_wqe_fifo_pop) begin
   i_wqe<=0;
   i_wqe[136+:5]<=pops==0 ? expected_qp : 5'd9;
   pops=pops+1;
  end
  if(o_wqe_fifo_push) begin
   if(o_wqe[136+:5]!==5'd9) $fatal(1,"unrelated QP WQE not preserved");
   pushes=pushes+1;
  end
  if(o_wqe_dropped) begin
   if(o_wqe_dropped_qpid!==expected_qp) $fatal(1,"wrong drop QP");
   drops=drops+1;
  end
  if(o_retransmit_wqe_bram_wen) begin
   if(o_retransmit_wqe_bram_idx!==expected_qp || o_retransmit_wqe_bram_din!=={expected_toggle,expected_payload}) $fatal(1,"retry metadata mismatch");
   commits=commits+1;
  end
 end
end
task exercise(input [23:0] psn,input [31:0] done_bytes,input integer entries,input integer stall);
 integer k;
 begin
  @(negedge core_clk);core_rstn=0;
  repeat(4) @(negedge core_clk);
  pops=0;pushes=0;drops=0;commits=0;
  expected_qp=5'd3;expected_psn=psn;expected_ssn=24'h9;
  expected_payload={psn,done_bytes};expected_toggle=done_bytes!=0;
  i_retransmit_qpid=expected_qp;i_psn_to_retry=expected_psn;i_ssn_to_retry=expected_ssn;
  i_retransmit_wqe_bram_din=expected_payload;i_retransmit_wqe_bram_dout=0;
  i_num_valid_entries=entries;i_qp_cq_hdptr=16'h2a;i_qp_sq_psn=24'h777;
  i_qp_stat_ssn=24'h88;i_halted=0;i_qp_cq_hdptr_valid=0;i_qp_sq_psn_valid=0;
  i_qp_stat_ssn_valid=0;i_qp_stat_ret_sq_psn_valid=0;i_prog_full_db_update_curr_sqptr=stall!=0;
  core_rstn=1;
  repeat(2) @(negedge core_clk);
  if(!o_retransmit_accepted || o_halt) $fatal(1,"idle interface mismatch");
  i_retransmit_reqd=1;@(negedge core_clk);i_retransmit_reqd=0;
  repeat(3) begin
   @(negedge core_clk);
   if(!o_halt || o_qp_sq_psn_wrn) $fatal(1,"halt handshake bypassed");
  end
  i_halted=1;
  repeat(3) @(negedge core_clk);
  i_qp_cq_hdptr_valid=1;i_qp_sq_psn_valid=1;i_qp_stat_ssn_valid=1;i_qp_stat_ret_sq_psn_valid=1;
  if(stall!=0) begin
   repeat(8) begin @(negedge core_clk);if(commits!=0) $fatal(1,"commit before doorbell release");end
   i_prog_full_db_update_curr_sqptr=0;
  end
  k=0;
  while((commits==0 || !o_retransmit_accepted) && k<150) begin @(negedge core_clk);k=k+1;end
  if(k==150) $fatal(1,"recovery did not finish");
  repeat(2) @(negedge core_clk);
  if(commits!=1 || pops!=entries || pushes!=(entries==2 ? 1:0) || drops!=(entries==2 ? 1:0)) $fatal(1,"FIFO scoreboard mismatch p=%0d push=%0d drop=%0d commits=%0d",pops,pushes,drops,commits);
  cases=cases+1;
  $display("CASE_PASS case=%0d psn=%h completed_bytes=%0d entries=%0d stall=%0d",cases,psn,done_bytes,entries,stall);
 end
endtask
initial begin
 fd=$fopen("retransmit_trace.csv","w");$fdisplay(fd,"cycle,state,halt,pop,push,drop,commit,rollback_psn");
 $dumpfile("retransmit.vcd");$dumpvars(0,tb);
 exercise(24'h101,0,0,0);
 exercise(24'h108,4096,2,0);
 exercise(24'h201,8192,2,1);
 exercise(24'hfffffe,0,0,1);
 $display("UNIT_PASS cases=%0d scope=retransmit_controller_only",cases);
 $fclose(fd);$finish;
end
initial begin #20000;$fatal(1,"global timeout");end
endmodule
