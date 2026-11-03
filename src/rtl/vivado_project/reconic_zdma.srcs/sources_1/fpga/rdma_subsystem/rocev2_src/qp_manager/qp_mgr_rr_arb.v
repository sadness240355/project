// /////////////////////////////////////////////////////////////////
// (c) Copyright 2019 - 2020 Xilinx, Inc. All rights reserved.	 
//   	                                						 
// This file contains confidential and proprietary information	 
// of Xilinx, Inc. and is protected under U.S. and	        	 
// international copyright and other intellectual property    	 
// laws.							                             
//   							                                 
// DISCLAIMER							                         
// This disclaimer is not a license and does not grant any	     
// rights to the materials distributed herewith. Except as	     
// otherwise provided in a valid license issued to you by	     
// Xilinx, and to the maximum extent permitted by applicable	
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND	     
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES	 
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING	 
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-	     
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and	     
// (2) Xilinx shall not be liable (whether in contract or tort,	 
// including negligence, or under any other theory of		     
// liability) for any loss or damage of any kind or nature	     
// related to, arising under or in connection with these	     
// materials, including for any direct, or any indirect,	    
// special, incidental, or consequential loss or damage		     
// (including loss of data, profits, goodwill, or any type of	 
// loss or damage suffered as a result of any action brought	 
// by a third party) even if such damage or loss was		     
// reasonably foreseeable or Xilinx had been advised of the	     
// possibility of the same.					                     
//   							                                 
// CRITICAL APPLICATIONS					                     
// Xilinx products are not designed or intended to be fail-	     
// safe, or for use in any application requiring fail-safe	     
// performance, such as life-support or safety devices or	     
// systems, Class III medical devices, nuclear facilities,	     
// applications related to the deployment of airbags, or any	  
// other applications that could lead to death, personal	      
// injury, or severe property or environmental damage		     
// (individually and collectively, "Critical			          
// Applications"). Customer assumes the sole risk and		     
// liability of any use of Xilinx products in Critical		     
// Applications, subject only to applicable laws and	  	     
// regulations governing limitations on product liability.	     
//   							                                 
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS	     
// PART OF THIS FILE AT ALL TIMES. 				                 
// ////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
//
// Filename        : qp_mgr_rr_arb.v
// Version         : v1.0
// Description     : Round Robin arbiter. This module simply moves a counter
// in a round robin fashion. All available queues are divided in to block of
// 8 qps for searchin the valid qp
// 
//
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
//-- Structure:
//		   -- qp_mgr_rr_arb.v
//-----------------------------------------------------------------------------
// Naming Conventions:
//      active low signals:                     "*_n"
//      clock signals:                          "clk", "clk_div#", "clk_#x" 
//      reset signals:                          "rst", "rst_n" 
//      parameters:                             "C_*" 
//      user defined types:                     "*_TYPE" 
//      state machine next state:               "*_ns" 
//      state machine current state:            "*_cs" 
//      combinatorial signals:                  "*_com" 
//      pipelined or register delay signals:    "*_d#" 
//      counter signals:                        "*cnt*"
//      clock enable signals:                   "*_ce" 
//      internal version of output port         "*_i"
//      device pins:                            "*_pin" 
//      component instantiations:               "<MODULE>I_<#|FUNC>
//-----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// Module Declaration
// 轮询仲裁器（round robin（rr）轮询仲裁）。目前仅用于QP SQ仲裁
// RDMA QP管理器中的轮询仲裁器模块，负责在多个QP之间进行公平的轮询仲裁，选择下一个要处理的QP。
///////////////////////////////////////////////////////////////////////////////
// 仲裁流程: 
// 1、检测QP可用性：监控所有QP的可用状态
// 
// 2、分组处理：将QP分成8个一组，先进行组级仲裁
// 
// 3、轮询机制：通过循环移位确保公平性
// 
// 4、两级仲裁：
//     组间仲裁：选择下一个有效的QP组
//     组内仲裁：在选定的组内选择具体的QP
// 
// 5、决策输出：最终输出选定的QP索引 sel_qp_next
///////////////////////////////////////////////////////////////////////////////

`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module qp_mgr_rr_arb #(
    parameter C_NUM_ELMNTS_TO_ARB = 32,  // 需要仲裁的元素的最大数量，32
    parameter C_ELMNT_PTR_WIDTH   = 5    // SSD编号指针宽度，5
) (
    // Clock and Reset
    input wire core_clk,  // Module Clock  
    input wire core_rst,  // Active high core reset

    // Configurations driven from register interface    
    //input   wire	[C_ELMNT_PTR_WIDTH :0]		    i_num_elements_enabled,	    // Number of QPs 

    // sq_arbiter 的接口
    input  wire                            i_arbitrate,            // 外部仲裁请求
    input  wire                            i_enable,               // 模块使能 --> 启用该模块是为了进行仲裁
    output wire                            o_arbitration_done,     // 完成仲裁
    output wire [C_ELMNT_PTR_WIDTH -1 : 0] o_elmnt_index,          // 选中的QP索引
    output wire [C_NUM_ELMNTS_TO_ARB -1:0] o_elmnt_index_one_hot,
    input  wire [C_NUM_ELMNTS_TO_ARB -1:0] i_elmnt_unavail         // 外部提供的不可用QP掩码
);

    //`include "../../rdma_macros.vh"

    // Common Declartion
    reg  [C_NUM_ELMNTS_TO_ARB -1:0] qp_avail;  // qp 可用掩码
    wire [C_NUM_ELMNTS_TO_ARB -1:0] qp_avail_combo;  // qp 可用掩码

    reg                             qp_avail_ff;  // 是否有QP可用？（qp可用掩码(qp_avail)任意位为1时该信号为1）；
    reg                             qp_avail_2ff;  // qp_avail_ff 延迟
    reg  [C_ELMNT_PTR_WIDTH -1 : 0] sel_qp;  // sel_qp=0表示从QP 0开始仲裁；arabitrated qp
    reg  [C_ELMNT_PTR_WIDTH -1 : 0] sel_qp_next;  // next arbitrated qp
    reg                             initial_condition;  // 是否进入初始状态？
    reg  [C_NUM_ELMNTS_TO_ARB -1:0] elmnt_index;  // one hot representation of selected qp available
    reg                             qp_next_base_addr_proc_en_2ff;
    reg  [C_ELMNT_PTR_WIDTH -1 : 0] sel_qp_ff;
    reg                             arbitrate_r;


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initial condition and arbitration done generation can be optimized if handshake mechinsum get updated or optimised
    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    reg                             arb_done;  // 仲裁完成；如果有任何 qp 可用，处理将在 1 个 clk 周期内完成
    genvar gi;
    integer                              i;


    //generate 
    //begin
    //if (C_NUM_ELMNTS_TO_ARB > 8)
    //begin
    /// if total qp >8
    reg     [C_NUM_ELMNTS_TO_ARB/8 -1:0] qp_grp_valid;  // set of 8 qp has any one or more queue available
    wire    [                       7:0] qp_avail_2d                                                                                             [C_NUM_ELMNTS_TO_ARB/8 -1:0];  // qp_avail_2d_combo 延迟一周期
    wire    [                       7:0] qp_avail_2d_combo                                                                                       [C_NUM_ELMNTS_TO_ARB/8 -1:0];
    reg     [C_NUM_ELMNTS_TO_ARB/8 -1:0] qp_grp_valid_shifted;  // list of qp grp valid after circulate shifti
    reg     [C_ELMNT_PTR_WIDTH -1-3 : 0] next_qp_grp_base_addr_proc;  // to be added in to current arbitrated qp index
    reg     [C_ELMNT_PTR_WIDTH -1-3 : 0] next_qp_grp_base_addr;  // to be added in to current arbitrated qp index
    //reg [7:0] qp_avail_of_next_qp_grp;                                       // queue valid for next qp group which is valid
    reg     [                       7:0] qp_avail_of_next_qp_grp_shifted;  // queue valid for next qp group which is valid
    wire    [                       7:0] qp_avail_of_curr_qp_grp;  // queue valid for next qp group which is valid
    reg     [                       2:0] nxt_qp_avail_of_curr_qp_grp;  // qp_available in current_qp_grp : current 1 qp served at lowest priority
    reg     [                       2:0] next_qp_grp_offset_addr;  // queue offset addr valid for next qp which is available of next qp group 
    //else qp is less than 8 or equal
    wire                                 arb_done_flush;  // to flush arbitrated value which become unavail in between process
    reg     [                       7:0] qp_avail_of_curr_qp_grp_shifted;
    reg     [                       2:0] qp_avail_of_qp_grp                                                                                      [ C_NUM_ELMNTS_TO_ARB/8-1:0];  // offset addresses of all qp grp
    wire    [                       2:0] qp_avail_of_next_qp_grp;  // selected next qp offset address

    wire                                 qp_seltected_pulse;  // qp index available as ouput
    wire                                 qp_next_base_addr_proc_en;
    reg                                  qp_next_base_addr_proc_en_ff;
    wire                                 arbitrate;  // 总仲裁信号
    wire                                 dummy_i_arbitrate;  // 虚拟的仲裁信号，用于触发重新仲裁
    reg                                  dummy_i_arbitrate_ff;
    //end
    //else
    //begin
    reg     [  C_NUM_ELMNTS_TO_ARB -1:0] qp_avail_shifted;  // Shifting all qp as per current qp value

    //end
    //endgenerate

    // 选中QP输出
    // -------------------------------------------------------------
    // o_elemnt_index : output ; arbitrated qp
    assign o_elmnt_index = sel_qp;

    // o_element_index_one_hot : arbitrated qp in one hot 
    // 独热编码输出
    always @(*) begin
        for (i = 0; i < C_NUM_ELMNTS_TO_ARB; i = i + 1) begin
            if (sel_qp == i && i_enable) elmnt_index[i] <= 1'b1;
            else elmnt_index[i] <= 1'b0;
        end
    end
    assign o_elmnt_index_one_hot = elmnt_index;
    // -------------------------------------------------------------

    // 可用于仲裁的 Qp
    // 寄存器存储QP可用状态
    always @(posedge core_clk) begin
        if (core_rst) qp_avail <= {C_NUM_ELMNTS_TO_ARB{1'b0}};
        else qp_avail <= qp_avail_combo;  // 更新可用QP掩码
    end
    // QP可用掩码（通过位取反QP不可用掩码）
    assign qp_avail_combo = ~i_elmnt_unavail;


    // 是否有QP可用？ 注册可用于仲裁的 QP
    always @(posedge core_clk) begin
        if (core_rst) qp_avail_ff <= {C_NUM_ELMNTS_TO_ARB{1'b0}};
        else qp_avail_ff <= |qp_avail;  // 检测是否有QP可用
    end


    // 优先级函数：从第 1 位至第 7 位选择输入数组 "in_val "中最先出现的位，
    // 其中第 1 位的优先级最高，第 7 位的优先级次之，第 0 位的优先级最低。




    // to circular shift all configured qp valid signal
    // 循环移位函数
    function [C_NUM_ELMNTS_TO_ARB -1:0] circ_shift_ttl_arb;
        input [C_NUM_ELMNTS_TO_ARB -1:0] shift_val;  // 要移位的值
        input [C_ELMNT_PTR_WIDTH -1:0] shifted_with;  // 移位量
        // shift_val 循环左移 shifted_with 位
        reg     [C_NUM_ELMNTS_TO_ARB -1:0] shifted_val;
        integer                            k;
        begin
            shifted_val = shift_val;
            for (k = 1; k < C_NUM_ELMNTS_TO_ARB; k = k + 1)
            if (shifted_with >= k) begin
                shifted_val = {shifted_val[0], shifted_val[C_NUM_ELMNTS_TO_ARB-1:1]};  // 左移 1 位
            end
            circ_shift_ttl_arb = shifted_val;
        end
    endfunction

    // to priortise among all configured qp valid signal ; afer circular shift
    // 在所有配置的 qp 有效信号中进行优先级排序；经过循环移位后
    // 优先级编码函数
    function [C_ELMNT_PTR_WIDTH : 0] priority_ttl_arb;
        input [C_NUM_ELMNTS_TO_ARB-1 : 0] in_val;
        // 从低位到高位扫描，找到最后一个1的位置（最高优先级）
        integer j;
        begin
            priority_ttl_arb = {C_ELMNT_PTR_WIDTH{1'b0}};
            for (j = 1; j < C_NUM_ELMNTS_TO_ARB; j = j + 1)
            if (in_val[j]) begin
                priority_ttl_arb = j;
            end
        end
    endfunction

    //output arbitration done -- 完成输出仲裁
    //assign o_arbitration_done = arb_done && (!arbitrate) && (!arb_done_flush);
    // 总仲裁信号合成
    assign arbitrate         = i_arbitrate || dummy_i_arbitrate_ff;
    // 虚拟仲裁信号，当仲裁已完成（arb_done=1）但当前选中的QP变为不可用（qp_avail[sel_qp]=0）时，触发虚拟仲裁，需要重新选择QP
    assign dummy_i_arbitrate = arb_done && !qp_avail[sel_qp];

    // 虚拟仲裁状态寄存器
    always @(posedge core_clk) begin
        if (core_rst) dummy_i_arbitrate_ff <= 1'b0;
        else if (dummy_i_arbitrate) dummy_i_arbitrate_ff <= !dummy_i_arbitrate_ff;  // 检测到虚拟仲裁需求时：翻转，产生一个时钟周期的脉冲
        else if (!dummy_i_arbitrate) dummy_i_arbitrate_ff <= 1'b0;  // 虚拟仲裁需求消失时：清零，准备下次检测
    end

    generate

        // 小规模架构 (≤8个QP)
        if (C_NUM_ELMNTS_TO_ARB <= 8) begin

            // selcted qp: arbitrated qp
            // 选择的 QP：仲裁的 QP
            // QP选择状态管理
            always @(posedge core_clk) begin
                if (core_rst) sel_qp <= {C_ELMNT_PTR_WIDTH{1'b0}};
                else if ((initial_condition && (!arb_done)) || arbitrate) begin  // (初始条件成立且仲裁未完成) 或 有仲裁请求时
                    sel_qp <= sel_qp_next;  // 将预计算的下一个QP索引 sel_qp_next 赋给当前选中QP
                end
            end

            // arbitraion_done: value selected after arbitration
            // arbitraion_done：仲裁后选择的值
            // 仲裁完成状态
            always @(posedge core_clk) begin
                if (core_rst) arb_done <= 1'b0;
                else if (initial_condition || arbitrate)  // 初始条件成立或有仲裁请求时
                    //arb_done <= |qp_avail_ff;
                    arb_done <= |qp_avail;  // 如果有任何可用QP (|qp_avail)，则设置仲裁完成
            end


            // First value to be pushed out -- 第一个被推出的值
            // 初始条件管理
            always @(posedge core_clk) begin
                if (core_rst) initial_condition <= 1'b1;
                else if (!(|qp_avail) && arbitrate) initial_condition <= 1'b1;  // 无可用QP且有仲裁请求时，等待QP变为可用
                else if (|qp_avail && arb_done) initial_condition <= 1'b0;  // 有可用QP且仲裁完成时，进入正常轮询状态
                else if (((!arb_done) && (qp_avail_ff))) initial_condition <= 1'b1;  // 仲裁未完成但之前有可用QP时，重新进行仲裁
            end


            // 循环移位函数 - 将当前选中QP移到最低优先级
            always @(*) begin
                // 第1步：将当前选中QP移到最低优先级（循环左移）
                qp_avail_shifted = circ_shift_ttl_arb(qp_avail, sel_qp);
                // 第2步：在移位后的掩码中找到第一个可用QP
                sel_qp_next      = priority_ttl_arb(qp_avail_shifted) + sel_qp;
            end
            // 仲裁完成信号
            assign o_arbitration_done = arb_done && (!arbitrate) && !dummy_i_arbitrate;

        end else begin  // current: 大规模架构 (>8个QP)

            // First value to be pushed out -- 第一个被推出的值
            // 初始条件管理
            always @(posedge core_clk) begin
                if (core_rst) initial_condition <= 1'b1;
                else if (!(|qp_avail) && arbitrate) initial_condition <= 1'b1;  // 无可用QP且有仲裁请求时，进入初始状态
                else if (|qp_avail && arb_done) initial_condition <= 1'b0;  // 有可用QP且仲裁完成时，不进入初始状态
                else if (((!arb_done) && (qp_avail_2ff))) initial_condition <= 1'b1;  // 仲裁未完成但2周期前有可用QP时，进入初始状态
            end

            assign o_arbitration_done = arb_done && (!arbitrate) && !dummy_i_arbitrate;


            // selcted qp: arbitrated qp
            // 选定的 QP：仲裁的 QP
            /*   always @(posedge core_clk)
	     begin
		if (core_rst)
		  sel_qp <= {C_ELMNT_PTR_WIDTH {1'b0}};
		else if (qp_next_base_addr_proc_en_ff || arbitrate)
		  begin
		     sel_qp <= sel_qp_next;		
		  end	
	     end */

            // QP选择状态管理
            always @(*) begin
                if (qp_next_base_addr_proc_en_2ff || arbitrate_r) begin  // 下一个QP基地址处理使能（2级延迟） 或 延迟1拍的仲裁信号
                    sel_qp = sel_qp_next;
                end else begin
                    sel_qp = sel_qp_ff;
                end
            end

            always @(posedge core_clk) arbitrate_r <= arbitrate;  // 延迟

            // pulse to detect 1st cycle to n cycle till arbitrate signal get asserted
            // after done to start curr_qp_offser_addr_calc

            always @(posedge core_clk) begin
                if (core_rst) begin
                    qp_avail_2ff <= {C_NUM_ELMNTS_TO_ARB{1'b0}};
                    sel_qp_ff    <= 'd0;
                end else begin
                    qp_avail_2ff <= qp_avail_ff;  // 延迟
                    sel_qp_ff    <= sel_qp;  // 延迟
                end
            end

            // QP选中脉冲 = 仲裁已完成 且 当前无仲裁请求
            assign qp_seltected_pulse = arb_done && !arbitrate;

            // Priority function : select the first one occurenc in input array "in_val
            // from bit 1:7; keeping first but on higest priority and 7 bit on second
            // lowest priority and 0th bit has lowest priority among all



            // Circ_shift funciton : circular left shift the shift_val with shifted_with
            // input value 
            //	   function [C_NUM_ELMNTS_TO_ARB/8 -1:0] circ_shift;
            //	      input  [C_NUM_ELMNTS_TO_ARB/8 -1:0] shift_val;
            //	      input [ C_ELMNT_PTR_WIDTH -1-3:0]   shifted_with;
            //	      reg [C_NUM_ELMNTS_TO_ARB/8 -1:0] 	  shifted_val;
            //	      integer 				  k;
            //	      begin
            //		 shifted_val = shift_val; 
            //		 for (k = 1; k < C_NUM_ELMNTS_TO_ARB/8 ; k=k+1)
            //		   if (shifted_with >= k)
            //		     begin	
            //			shifted_val = {shifted_val[0],shifted_val[C_NUM_ELMNTS_TO_ARB/8 -1:1]}; // circular left shift by 1
            //		     end	
            //		 circ_shift = shifted_val;
            //	      end	
            //	   endfunction

            function [C_NUM_ELMNTS_TO_ARB/8 -1:0] circ_shift;
                input [C_NUM_ELMNTS_TO_ARB/8 -1:0] shift_val;
                input [C_ELMNT_PTR_WIDTH -1-3:0] shifted_with;
                reg     [C_NUM_ELMNTS_TO_ARB/8 -1:0] shifted_val;
                integer                              k;
                reg     [C_NUM_ELMNTS_TO_ARB/8 -1:0] mux_in      [C_NUM_ELMNTS_TO_ARB/8 -1:0];
                begin
                    mux_in[0] = shift_val;  // mux_in[0] = 1101 (原始值shift_val)
                    for (k = 1; k < C_NUM_ELMNTS_TO_ARB / 8; k = k + 1) begin
                        mux_in[k] = {mux_in[k-1][0], mux_in[k-1][C_NUM_ELMNTS_TO_ARB/8-1:1]};  // circular left shift by 1
                    end
                    circ_shift = mux_in[shifted_with];  // 根据 shifted_with 选择对应的移位结果，若是 shifted_with = 2，则返回mux_in[2] = 0111 (左移2位)
                end
            endfunction

            function [2:0] priority_8_in;
                input [7 : 0] in_val;
                integer j;
                begin
                    priority_8_in = {3{1'b0}};
                    for (j = 0; j < 8; j = j + 1)
                    if (in_val[j]) begin
                        priority_8_in = j;  // overide privious index
                    end
                end
            endfunction

            function [7:0] circ_shift_8_in;
                input [7:0] shift_val;
                input [2:0] shifted_with;
                reg     [7:0] shifted_val;
                integer       k;
                begin
                    shifted_val = shift_val;
                    for (k = 1; k < 8; k = k + 1)
                    if (shifted_with >= k) begin
                        shifted_val = {shifted_val[0], shifted_val[8-1:1]};  // circular left shift by 1
                    end
                    circ_shift_8_in = shifted_val;
                end
            endfunction

            function [C_ELMNT_PTR_WIDTH -1-3 : 0] priority_param;
                input [C_NUM_ELMNTS_TO_ARB/8-1 : 0] in_val;  // 0100
                integer j;
                begin
                    priority_param = {(C_ELMNT_PTR_WIDTH - 3) {1'b0}};
                    for (j = 1; j < C_NUM_ELMNTS_TO_ARB / 8; j = j + 1) begin
                        if (in_val[j]) begin
                            priority_param = j;  // overide privious index
                        end
                    end
                end
            endfunction


            // Architecture for more than 8 to 256 qp
            // 仲裁完成状态
            always @(posedge core_clk) begin
                if (core_rst) arb_done <= 1'b0;
                else if (initial_condition || arbitrate)
                    //arb_done <= qp_avail[sel_qp_next];
                    arb_done <= qp_avail_ff;  // 使用流水线延迟的可用性信号，以匹配时序
            end

            assign arb_done_flush = arb_done && i_elmnt_unavail[sel_qp];

            // -------------------------------------------------------------
            // tips: qp_avail_2d 比 qp_avail_2d_combo 延迟一周期
            // QP分组管理
            // qp 组 2d 阵列，共(C_NUM_QP / 8) = 4行， 每行8个QP(8bit)
            // 将QP分成每组8个
            for (gi = 0; gi < C_NUM_ELMNTS_TO_ARB / 8; gi = gi + 1) begin
                assign qp_avail_2d[gi] = qp_avail[(gi+1)*8-1:(gi*8)];  // 每组QP的可用性
            end

            // qp 组 2d 阵列，共(C_NUM_QP / 8) = 4行， 每行8个QP(8bit)
            // 将QP分成每组8个
            for (gi = 0; gi < C_NUM_ELMNTS_TO_ARB / 8; gi = gi + 1) begin
                assign qp_avail_2d_combo[gi] = qp_avail_combo[(gi+1)*8-1:(gi*8)];  // 每组QP的可用性
            end
            // -------------------------------------------------------------

            // 组级可用性检测：检查每组中是否有可用的QP
            for (gi = 0; gi < C_NUM_ELMNTS_TO_ARB / 8; gi = gi + 1) begin
                always @(posedge core_clk) begin
                    if (core_rst) qp_grp_valid[gi] <= 'b0;
                    else qp_grp_valid[gi] <= |qp_avail_2d_combo[gi];  // 如果组内有任何QP可用，则该组可用
                end
            end

            // -------------------------------------------------------------
            // 组级循环左移
            // 若sel_qp = 8（第一组），qp_grp_valid = 4'b0010, 则左移1位（sel_qp[4:3] = 1）后 qp_grp_valid_shifted = 4'b0100
            always @(posedge core_clk) begin
                if (core_rst) qp_grp_valid_shifted <= {C_ELMNT_PTR_WIDTH{1'b0}};
                else if (qp_seltected_pulse || qp_next_base_addr_proc_en) qp_grp_valid_shifted <= circ_shift(qp_grp_valid, sel_qp[C_ELMNT_PTR_WIDTH-1:3]);
            end

            // 下一个QP基地址处理使能 = 当前有可用QP 且 上一拍无可用QP，检测QP可用性从无到有的跳变。
            assign qp_next_base_addr_proc_en = |qp_avail && !(qp_avail_ff);

            always @(posedge core_clk) begin
                if (core_rst) begin
                    qp_next_base_addr_proc_en_ff  <= 1'b0;
                    qp_next_base_addr_proc_en_2ff <= 1'b0;
                end else begin
                    qp_next_base_addr_proc_en_ff  <= qp_next_base_addr_proc_en;  // 1级延迟
                    qp_next_base_addr_proc_en_2ff <= qp_next_base_addr_proc_en_ff;  // 2级延迟
                end
            end

            // next qp base addr calc and register -- 下一个 QP 基本地址计算和寄存器
            always @(*)
                //	begin
                //   if (core_rst)
                //   next_qp_grp_base_addr_proc <= {{C_ELMNT_PTR_WIDTH-3}{1'b0}};
                //else if  (initial_condition || (arbitrate && arb_done))
                // else

                // 选择下一个有效组
                next_qp_grp_base_addr_proc = priority_param(
                    qp_grp_valid_shifted
                );  // + sel_qp[C_ELMNT_PTR_WIDTH-1:3] ;   
            //	end

            // 计算下一个组的绝对地址
            always @(*) next_qp_grp_base_addr = next_qp_grp_base_addr_proc + sel_qp[C_ELMNT_PTR_WIDTH-1:3];
            // -------------------------------------------------------------

            // Priority encoder on all qp grp to know offeset address of all qp grp
            for (gi = 0; gi < C_NUM_ELMNTS_TO_ARB / 8; gi = gi + 1) begin
                always @(posedge core_clk) begin
                    if (core_rst) qp_avail_of_qp_grp[gi] <= 3'b0;
                    //		else if(initial_condition || (arb_done && arbitrate))
                    else if (qp_seltected_pulse || qp_next_base_addr_proc_en) qp_avail_of_qp_grp[gi] <= priority_8_in(qp_avail_2d[gi]);
                end
            end

            // Mux to select particular next qp offset address
            assign qp_avail_of_next_qp_grp = qp_avail_of_qp_grp[next_qp_grp_base_addr];

            // Current offset address calc Process

            // 组内QP仲裁
            // -------------------------------------------------------------
            // Mux to select current qp grp out of all qp grps
            // 获取当前组的QP可用性
            assign qp_avail_of_curr_qp_grp = qp_avail_2d[(sel_qp[C_ELMNT_PTR_WIDTH-1:3])];

            //Pipe line stage
            // 组内循环移位
            always @(posedge core_clk) begin
                if (core_rst) qp_avail_of_curr_qp_grp_shifted <= 8'b0;
                else if (qp_seltected_pulse) qp_avail_of_curr_qp_grp_shifted <= qp_avail_of_curr_qp_grp << (8 - sel_qp[2:0]);
            end
            // 组内下一个QP选择
            always @(*) nxt_qp_avail_of_curr_qp_grp = priority_8_in(qp_avail_of_curr_qp_grp_shifted) + sel_qp[2:0];

            /* 	   always @(*)
				   begin
				  if(nxt_qp_avail_of_curr_qp_grp != sel_qp[2:0])
				    begin  
				       next_qp_grp_offset_addr = nxt_qp_avail_of_curr_qp_grp;
				       sel_qp_next = {sel_qp[C_ELMNT_PTR_WIDTH-1:3],next_qp_grp_offset_addr};
				    end
				  else
				    begin
				       // next_qp_grp_offset_addr= priority_8_in(qp_avail_of_next_qp_grp); 
				       next_qp_grp_offset_addr =   qp_avail_of_next_qp_grp;                                // selected qp in next qp grp
				       sel_qp_next = {next_qp_grp_base_addr,next_qp_grp_offset_addr};
				    end
				   end // always @ begin */
            // 下一个QP决策
            // -------------------------------------------------------------
            always @(posedge core_clk) begin : Inserting_pipeline_stage
                if (nxt_qp_avail_of_curr_qp_grp != sel_qp[2:0]) begin
                    // 当前组内还有其他可用QP
                    sel_qp_next <= {sel_qp[C_ELMNT_PTR_WIDTH-1:3], nxt_qp_avail_of_curr_qp_grp};
                end else begin
                    // 当前组内无其他可用QP，跳转到下一个有效组
                    sel_qp_next <= {next_qp_grp_base_addr, qp_avail_of_next_qp_grp};
                end
            end
            // -------------------------------------------------------------

        end  // else: !if(C_NUM_ELMNTS_TO_ARB <= 8)
    endgenerate


endmodule  // qp_mgr_rr_arb
