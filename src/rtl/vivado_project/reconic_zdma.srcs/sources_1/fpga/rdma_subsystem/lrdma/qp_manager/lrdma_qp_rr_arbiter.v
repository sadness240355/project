`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_qp_rr_arbiter — QP SQ 轮询仲裁器（Round-Robin Arbiter）
//
// 【模块概述】
// 在多个 QP 的 SQ 之间进行公平轮询仲裁，确保每个 QP 都能获得发送机会。
// 通过 i_elmnt_unavail 掩码获知哪些 QP 不可用（SQ 为空或已禁用）。
//
// 【仲裁算法】
// 核心：循环移位 + 优先级编码。把已选中的 QP 旋转到最低优先级位置，然后
// 在旋转后的掩码中选出优先级最高的可用 QP。
//
// 【两种架构（由 generate 根据 C_NUM_ELMNTS_TO_ARB 选择）】
// 1. 小规模（≤ 8 QP）：单级仲裁
//    - circ_shift_ttl_arb（循环移位）+ priority_ttl_arb（优先级编码）
// 2. 大规模（> 8 QP）：两级分层仲裁
//    - 所有 QP 分成每组 8 个（如 32 QP = 4 组）
//    - 组间：qp_grp_valid + circ_shift + priority_param 选择有效组
//    - 组内：circ_shift_8_in + priority_8_in 选择具体 QP
//    该分层设计将关键路径从 O(N) 降到 O(sqrt(N))
//
// 【特性】
// - 流水线设计：仲裁结果通过 sel_qp_ff 寄存器输出
// - dummy_i_arbitrate 机制：当已选定 QP 运行时变为不可用，自动触发重选
// - initial_condition 标志处理首次启动
// - o_arbitration_done 在仲裁完成且无新请求时有效
///////////////////////////////////////////////////////////////////////////////

`timescale 1ps / 1ps
module lrdma_qp_rr_arbiter #(
    parameter C_NUM_ELMNTS_TO_ARB = 32,  // 需要仲裁的元素的最大数量，32
    parameter C_ELMNT_PTR_WIDTH   = 5    // 元素编号/队列索引指针宽度，5
) (
    // Clock and Reset
    input core_clk,  // Module Clock  
    input core_rst,  // Active high core reset

    // sq_arbiter 的接口
    input                             i_arbitrate,            // 外部仲裁请求
    input                             i_enable,               // 模块使能 --> 启用该模块是为了进行仲裁
    output                            o_arbitration_done,     // 完成仲裁
    output [C_ELMNT_PTR_WIDTH -1 : 0] o_elmnt_index,          // 选中的QP索引
    output [C_NUM_ELMNTS_TO_ARB -1:0] o_elmnt_index_one_hot,
    input  [C_NUM_ELMNTS_TO_ARB -1:0] i_elmnt_unavail         // 外部提供的不可用QP掩码
);

    //`include "lrdma_defs.vh"

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
    reg     [C_ELMNT_PTR_WIDTH -1-3 : 0] next_qp_grp_base_addr_proc;  // 组级仲裁得到的相对组索引
    reg     [C_ELMNT_PTR_WIDTH -1-3 : 0] next_qp_grp_base_addr;  // 下一组的绝对组索引
    //reg [7:0] qp_avail_of_next_qp_grp;                                       // queue valid for next qp group which is valid
    wire    [                       7:0] qp_avail_of_curr_qp_grp;  // queue valid for next qp group which is valid
    reg     [                       2:0] nxt_qp_avail_of_curr_qp_grp;  // 本组内下一可用 QP 偏移，已服务 QP 作为最低优先级
    //else qp is less than 8 or equal
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
    reg     [  C_NUM_ELMNTS_TO_ARB -1:0] qp_avail_shifted;  // 按已选 QP 旋转后的可用掩码

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


    // 优先级函数：输入一个 one-hot/multi-hot 掩码，返回“位置最高的那个 1 的下标”。
    // 扫描 j 从 1 走到 N-1，每遇到 in_val[j]==1 就覆写 priority_ttl_arb=j，最后一次赋值赢。
    // 返回值随后与 sel_qp 相加得到下一个被选中的 QP 索引。




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
    // 在循环移位后的掩码中挑出优先级最高的可用位。
    // 优先级编码函数
    function [C_ELMNT_PTR_WIDTH : 0] priority_ttl_arb;
        input [C_NUM_ELMNTS_TO_ARB-1 : 0] in_val;
        // 扣描下标从低到高，最后一个 1 的位置优先级最高
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
    // 虚拟仲裁信号，当仲裁已完成（arb_done=1）但已选中的QP变为不可用（qp_avail[sel_qp]=0）时，触发虚拟仲裁，需要重新选择QP
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
                    sel_qp <= sel_qp_next;  // 将预计算的下一个QP索引 sel_qp_next 赋给选中QP
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
                else if (((!arb_done) && (qp_avail_ff))) initial_condition <= 1'b1;  // 仲裁未完成且上一拍有可用 QP 时，重新进行仲裁
            end


            // 循环移位函数 - 将已选中QP移到最低优先级
            always @(*) begin
                // 第1步：将已选中QP移到最低优先级（循环左移）
                qp_avail_shifted = circ_shift_ttl_arb(qp_avail, sel_qp);
                // 第2步：在移位后的掩码中找到第一个可用QP
                sel_qp_next      = priority_ttl_arb(qp_avail_shifted) + sel_qp;
            end
            // 仲裁完成信号
            assign o_arbitration_done = arb_done && (!arbitrate) && !dummy_i_arbitrate;

        end else begin  // 大规模架构 (>8个QP)

            // First value to be pushed out -- 第一个被推出的值
            // 初始条件管理
            always @(posedge core_clk) begin
                if (core_rst) initial_condition <= 1'b1;
                else if (!(|qp_avail) && arbitrate) initial_condition <= 1'b1;  // 无可用QP且有仲裁请求时，进入初始状态
                else if (|qp_avail && arb_done) initial_condition <= 1'b0;  // 有可用QP且仲裁完成时，不进入初始状态
                else if (((!arb_done) && (qp_avail_2ff))) initial_condition <= 1'b1;  // 仲裁未完成但2周期前有可用QP时，进入初始状态
            end

            assign o_arbitration_done = arb_done && (!arbitrate) && !dummy_i_arbitrate;

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

            // QP选中脉冲 = 仲裁已完成 且无仲裁请求
            assign qp_seltected_pulse = arb_done && !arbitrate;

            // Priority function : select the first one occurenc in input array "in_val
            // from bit 1:7; keeping first but on higest priority and 7 bit on second
            // lowest priority and 0th bit has lowest priority among all

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

            // 下一个QP基地址处理使能 = 本拍有可用QP 且上一拍无可用QP，检测QP可用性从无到有的跳变。
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
            // Mux to select selected qp grp out of all qp grps
            // 获取本组的QP可用性
            assign qp_avail_of_curr_qp_grp = qp_avail_2d[(sel_qp[C_ELMNT_PTR_WIDTH-1:3])];

            //Pipe line stage
            // 组内循环移位
            always @(posedge core_clk) begin
                if (core_rst) qp_avail_of_curr_qp_grp_shifted <= 8'b0;
                else if (qp_seltected_pulse) qp_avail_of_curr_qp_grp_shifted <= qp_avail_of_curr_qp_grp << (8 - sel_qp[2:0]);
            end

            // 组内下一个QP选择
            always @(*) nxt_qp_avail_of_curr_qp_grp = priority_8_in(qp_avail_of_curr_qp_grp_shifted) + sel_qp[2:0];

            // 下一个QP决策
            // -------------------------------------------------------------
            always @(posedge core_clk) begin : Inserting_pipeline_stage
                if (nxt_qp_avail_of_curr_qp_grp != sel_qp[2:0]) begin
                    // 本组内还有其他可用QP
                    sel_qp_next <= {sel_qp[C_ELMNT_PTR_WIDTH-1:3], nxt_qp_avail_of_curr_qp_grp};
                end else begin
                    // 本组内无其他可用QP，跳转到下一个有效组
                    sel_qp_next <= {next_qp_grp_base_addr, qp_avail_of_next_qp_grp};
                end
            end
            // -------------------------------------------------------------

        end  // else: !if(C_NUM_ELMNTS_TO_ARB <= 8)
    endgenerate


endmodule  // lrdma_qp_rr_arbiter
