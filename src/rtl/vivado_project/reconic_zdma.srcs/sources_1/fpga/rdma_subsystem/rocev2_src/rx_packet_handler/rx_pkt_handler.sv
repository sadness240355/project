//-----------------------------------------------------------------------------
// rx_pkt_handler.v
//-----------------------------------------------------------------------------
//
// Filename        : rx_pkt_handler.v
// Version         : v1.0
// Description     : 处理所有收到的 IB 数据包
// Verilog-Standard: Verilog'2001
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
// 处理所有收到的 IB 数据包
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps

module rx_pkt_handler #(
    parameter C_SIM_DEBUG = 0,  // 0
    parameter C_EN_INITIATOR_LITE = 0,  // 1
    parameter C_M_AXI_ADDR_WIDTH = 32,  // 32
    parameter C_M_AXI_AWUSER_WIDTH = 16,  // 32
    parameter C_NUM_QP = 256,  // 32
    parameter C_QP_INDX_WIDTH = 8,  // 5
    parameter C_NUM_PD = 256,  // 2048
    parameter C_PD_INDX_WIDTH = 8,  // 11
    parameter C_OSQ_PSN_WIDTH = 8,  // 16
    parameter NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN = 0,  // 0
    parameter C_EN_DEBUG_PORTS = 0,  // 0
    parameter C_EN_IETH_IMMDT = 1  // 1
) (
    // System Clock & Reset
    input wire core_clk,
    input wire core_rst_n,

    // 以太网控制器接口，AXI-Stream数据包输入
    input wire         s_axis_tvalid,
    input wire [511:0] s_axis_tdata,
    input wire [ 63:0] s_axis_tkeep,
    input wire         s_axis_tlast,
    input wire [  0:0] s_axis_tuser,

    // XRNIC register interface
    input wire                       i_xrnic_en,       // XRNIC interface enable from XRNIC registers
    input wire [C_QP_INDX_WIDTH-1:0] i_num_qp_enabled, // 活动的 QP 数量

    input wire [ 47:0] i_mac_node_addr,   // 本地 (ERNIC) 设备的 MAC 地址
    input wire [ 31:0] i_ipv4_node_addr,  // 本地 (ERNIC) 设备的 IPv4 地址
    input wire [127:0] i_ipv6_node_addr,  // 本地 (ERNIC) 设备的 IPv6 地址

    input wire [4:0] i_rnr_nak_tval,    // RNR NACK 传出数据包的超时值
    input wire [2:0] i_rnr_nak_rst_val, // 最大 RNR 重试次数

    input wire [1:0] i_flow_credits,  // 2'b00传入

    // 全0传入
    // XRNIC 接口旁路功能
    input  wire                          i_depkt_bypass_en,   // 启用 XRNIC 接口旁路功能
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_bypass_buf_ba,     // 旁路数据包队列基地址
    input  wire [                  15:0] i_bypass_num_bufs,   // 旁路数据包队列深度
    input  wire [                  15:0] i_bypass_buffer_sz,  // 旁路队列元素缓冲区大小（字节
    output wire [                  15:0] o_bypass_buf_wrptr,  // 旁路数据包队列当前写入索引

    // 传入req数据包错误缓冲区配置
    input  wire                          i_req_err_buf_en,         // XRNIC Configuration寄存器 -- [5] Error buffer enable
    input  wire                          i_req_err_buf_ovr_wr_en,  // Enables the error buffer overwriting
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_req_err_pkt_buf_ba,     // Error logging queue base address  // tip:错误缓冲区的基地址
    input  wire [                  15:0] i_req_err_num_bufs,       // Error logging queue depth // tip:错误缓冲区的数量
    input  wire [                  15:0] i_req_err_buffer_sz,      // Error logging queue element buffer size in bytes // tip:每个错误缓冲区的大小
    output wire [                  15:0] o_req_err_buf_wrptr,      // Error packet queue current write index   

    // 传入rsp数据包错误缓冲区配置
    input  wire                          i_rsp_err_buf_en,         // XRNIC Configuration寄存器 -- [5] Error buffer enable
    input  wire                          i_rsp_err_buf_ovr_wr_en,  // Enables the error buffer overwriting
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_rsp_err_pkt_buf_ba,     // Error logging queue base address
    input  wire [                  15:0] i_rsp_err_num_bufs,       // Error logging queue depth
    input  wire [                  15:0] i_rsp_err_buffer_sz,      // Error logging queue element buffer size in bytes
    output wire [                  15:0] o_rsp_err_buf_wrptr,      // Error packet queue current write index   

    // 传入数据包的错误状态队列配置
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_in_errsts_q_ba,    // 传入数据包错误状态队列的基地址
    input  wire [                  15:0] i_in_errsts_q_sz,    // 传入数据包错误状态队列的大小
    output wire [                  15:0] o_in_errsts_q_wrptr,

    // 各数据包统计
    output wire [15:0] o_inc_send_cnt,      // Incoming send request packets count：传入“发送请求“数据包计数
    output wire [15:0] o_inc_acknak_cnt,    // Incoming acknowledgement packets count：传入“确认“数据包计数，包括 ACK、 NAK 和 RNR-NAK
    output wire [15:0] o_inc_rresp_cnt,     // Incoming read response packets count：传入“读响应”数据包计数
    output wire [15:0] o_inc_mad_cnt,       // Incoming QP1 packet count：传入 QP1 数据包计数，即“MAD“数据包
    output reg  [15:0] o_inc_inv_pkt_cnt,   // Incoming invalid packet count：传入无效数据包计数
    output wire [15:0] o_inc_dup_pkt_cnt,   // Incoming duplicate packet count：传入重复数据包计数
    output wire [15:0] o_inc_nack_pkt_cnt,  // Incoming NAK packet count：传入 NAK 数据包计数，包括 NAK 和 RNR-NAK
    output wire [15:0] o_out_nack_pkt_cnt,  // Outgoing NAK packet count：传出 NAK 数据包计数
    output wire [31:0] pkt_all_cnt,         // all packet count：收到的所有数据包计数，丢弃的也计数
    output wire [31:0] pkt_disc_cnt,        // discarded packet count：丢弃数据包计数

    //input wire  [C_NUM_QP-1:0]                 i_qp_enable,
    input wire                       i_qp_enable_wr_en,
    input wire [C_QP_INDX_WIDTH-1:0] i_qp_enable_wr_idx,
    input wire                       i_qp_enable_wr_data,

    output wire [ 7:0] o_last_in_req_pkt_opcode,
    output wire [ 7:0] o_last_in_req_pkt_qpid,
    output wire [15:0] o_last_in_req_pkt_psn,

    output wire [ 7:0] o_last_in_rsp_pkt_opcode,
    output wire [ 7:0] o_last_in_rsp_pkt_qpid,
    output wire [15:0] o_last_in_rsp_pkt_psn,

    output wire [                7:0] o_last_in_pkt_opcode,
    output wire [C_QP_INDX_WIDTH-1:0] o_last_in_pkt_qpid,
    output wire [               15:0] o_last_in_pkt_psn,

    //output wire [C_NUM_QP-1:0]                 o_rq_full,                   // Receive request packet queue full
    output wire [C_QP_INDX_WIDTH-1:0] o_rq_full_idx,
    output wire                       o_rq_full_wr_en,
    output wire                       o_rq_full_val,
    output wire [C_QP_INDX_WIDTH-1:0] o_rq_full_reset_idx,
    output wire                       o_rq_full_reset_wr_en,
    output wire                       o_rq_full_reset_val,
    //output wire [C_NUM_QP-1:0]                 o_qp_fatal,                  // QPs those are stalled due to fatal error
    output wire                       o_qp_fatal_wr_en,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_fatal_wr_idx,
    output wire                       o_qp_fatal_wr_data,
    //input  wire [C_NUM_QP-1:0]                 i_clr_qp_fatal,              // removes stall condition on QPs
    input  wire                       i_clr_qp_fatal,         // removes stall condition on QPs
    input  wire [C_QP_INDX_WIDTH-1:0] i_clr_qp_fatal_idx,     // removes stall condition on QPs

    input  wire [               15:0] i_rq_pi_db_hw_hndshk,           // RQ PI Door Bell
    input  wire [               13:0] i_connect_io_qp_rq_pi_db_wptr,  // Connect IO QP DB address  
    input  wire [C_QP_INDX_WIDTH-1:0] i_connect_io_qpid,              // Connect IO QP DB address  
    input  wire                       i_hw_hndshk_disable_to_0,       // pulse indicates HWHNDSHK_DIS set 0  
    output wire                       o_connect_io_qp_rq_pi_db_rdy,   // Ready indicates can accept another

    //input  wire [C_NUM_QP -1:0]                qp_under_recovery,
    input wire                       qp_under_recovery_wr_en,
    input wire [C_QP_INDX_WIDTH-1:0] qp_under_recovery_wr_idx,
    input wire                       qp_under_recovery_wr_data,

    // Side-band RQ DB interface
    // XRNIC 侧边 RQ DB 接口
    output wire [31:0] o_rq_db_data,
    output wire [12:0] o_rq_db_addr,
    output wire        o_rq_db_data_valid,
    input  wire        i_rq_db_data_ack,    // ready

    input  wire [15:0] i_xrnic_pause_on,
    input  wire [15:0] i_xrnic_pause_off,
    // Interrupt controller interface
    input  wire        i_pkt_valdn_err_intr_en,   // Enable for Inbound packet header validation interrupt   
    output wire        o_pkt_valdn_err_intr,      // Inbound packet header validation error interrupt
    output wire        o_pkt_valdn_err_intr_sts,  // Inbound packet header validation error interrupt status flag
    input  wire        i_clr_pkt_valdn_err_intr,  // Clear Inbound packet header validation error interrupt

    input  wire i_mad_pkt_rcvd_intr_en,   // Enable to MAD packet received interrupt  
    output wire o_mad_pkt_rcvd_intr,      // MAD packet received interrupt
    output wire o_mad_pkt_rcvd_intr_sts,  // MAD packet received interrupt status flag
    input  wire i_clr_mad_pkt_rcvd_intr,  // Clear MAD packet received interrupt

    input  wire i_bypass_pkt_rcvd_intr_en,   // Enable to bypass packet received interrupt 
    output wire o_bypass_pkt_rcvd_intr,      // Incoming bypass packet received interrupt
    output wire o_bypass_pkt_rcvd_intr_sts,  // Incoming bypass packet received interrupt status flag
    input  wire i_clr_bypass_pkt_rcvd_intr,  // Clear Incoming bypass packet received interrupt    

    input  wire i_rnr_nack_gen_intr_en,
    output wire o_rnr_nack_gen_intr,
    output wire o_rnr_nack_gen_intr_sts,
    input  wire i_clr_rnr_nack_gen_intr,

    input  wire i_fatal_err_intr_en,
    output wire o_fatal_err_intr,
    output wire o_fatal_err_intr_sts,
    input  wire i_clr_fatal_err_intr,

    input  wire                       i_qp_pkt_rcvd_intr_en,           // Interrupt enable for qp_pkt_rcvd_intr           
    output wire                       o_qp_pkt_rcvd_intr,              // Interrupt to assert when admin qp packet received
    //output wire [C_NUM_QP-1:0]                 o_qp_pkt_rcvd_intr_sts,      // qp_pkt_rcvd_intr interrupt status bit             
    //input  wire [C_NUM_QP-1:0]                 i_clr_qp_pkt_rcvd_intr,      // Clear for qp_pkt_rcvd_intr
    output wire [               31:0] o_qp_pkt_rcvd_intr_sts,
    output wire [               31:0] o_qp_pkt_rcvd_intr_sts_wr_en,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_pkt_rcvd_intr_sts_wr_addr,
    input  wire [               31:0] i_clr_qp_pkt_rcvd_intr,
    input  wire [C_QP_INDX_WIDTH-1:0] i_clr_qp_pkt_rcvd_intr_wr_addr,

    // CNP related signals
    input  wire i_cnp_sched_intr_en,
    output wire o_cnp_sched_intr,
    output wire o_cnp_sched_intr_sts,
    input  wire i_clr_cnp_sched_intr,

    //output reg  [C_NUM_QP-1:0]                 o_cnp_sched_reg,
    //input  wire [C_NUM_QP-1:0]                 i_clr_cnp_sched,
    output reg  [               31:0] o_cnp_sched_wr_data,
    output reg  [C_QP_INDX_WIDTH-1:0] o_cnp_sched_wr_addr,
    output reg  [               31:0] o_cnp_sched_wr_en,
    input  wire [               31:0] i_clr_cnp_sched,
    input  reg  [C_QP_INDX_WIDTH-1:0] i_clr_cnp_sched_wr_addr,

    input wire [3:0] i_cnp_timeout_reg_1,  // Reserved
    input wire [3:0] i_cnp_timeout_reg_2,  // Reserved

    //output reg  [C_NUM_QP-1:0]                 o_cnp_pkt_qp_bitmap,    
    output reg [C_QP_INDX_WIDTH-1:0] o_cnp_pkt_rcvd_wr_idx,
    output reg                       o_cnp_pkt_rcvd_wr_en,
    output reg                       o_cnp_pkt_rcvd_wr_data,

    output reg [15:0] o_cnp_rx_pkt_cnt,

    // Response Handler interface -1
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_index_mpsnbuf,  // Max PSN queue (QP) index

    output wire o_max_epsn_req,
    input  wire i_max_epsn_vld,

    //input  wire [C_NUM_QP-1:0]                 i_mpsnbuf_empty,             // Max PSN queue empty
    input  wire                       i_mpsnbuf_empty_set_valid,
    input  wire [C_QP_INDX_WIDTH-1:0] i_mpsnbuf_empty_set_id,
    input  wire                       i_mpsnbuf_empty_reset_valid,
    input  wire [C_QP_INDX_WIDTH-1:0] i_mpsnbuf_empty_reset_id,
    //input  wire [C_NUM_QP-1:0]                 i_qp_retried,  
    input  wire                       i_qp_retried_set_valid,
    input  wire [C_QP_INDX_WIDTH-1:0] i_qp_retried_set_id,
    input  wire                       i_qp_retried_reset_valid,
    input  wire [C_QP_INDX_WIDTH-1:0] i_qp_retried_reset_id,
    output wire                       o_mpsnbuf_pop,                // Pop first element of Max PSN queue

    input wire [   C_OSQ_PSN_WIDTH-1:0] i_os_rdma_rd_resp_psn_7_0,    // PSN of expecting RDMA Read response
    input wire [C_M_AXI_ADDR_WIDTH-1:0] i_os_rdma_rd_resp_dest_addr,  // Destination address of expected RDMA Read response
    input wire [                  31:0] i_os_rdma_rd_resp_length,     // Length of expected RDMA read response

    output wire [C_QP_INDX_WIDTH-1:0] o_qp_index_acknack,    // QP Id for which ACKNOWLEDGE received
    output wire                       o_acknack_valid,       // valid pulse for this group of signals
    output wire [               23:0] o_acknacked_psn,       // BTH.PSN in ACKNOWLEDGE (one minus for read-first/middle) 
    output wire [                7:0] o_acknacked_syndrome,  // AETH.Syndrome in ACKNOWLEDGE  
    output wire [               23:0] o_acknacked_msn,       // AETH.MSN in ACKNOWLEDGE 
    output wire [                1:0] o_acknacked_opcode,    // 01:read-first/10:read-middle/11:read-last/only/00-acknowledge  
    output wire [               31:0] o_qp_timeout_per_qp,

    //input  wire [C_NUM_QP -1: 0]               i_osq_empty,
    input wire                                 i_osq_empty_set_valid,
    input wire [          C_QP_INDX_WIDTH-1:0] i_osq_empty_set_id,
    input wire                                 i_osq_empty_reset_valid,
    input wire [          C_QP_INDX_WIDTH-1:0] i_osq_empty_reset_id,
    //input  wire [C_NUM_QP -1: 0]               i_osq_nacked,
    input wire [(4*C_QP_INDX_WIDTH+4) - 1 : 0] i_osq_nacked,

    output wire                          o_rd_rsp_wr_cmpltd,
    output wire [   C_QP_INDX_WIDTH-1:0] o_rsp_qp_id,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_rsp_dest_addr_for_hw_hndshk,
    // WQE Processor interface -1 
    input  wire [   C_QP_INDX_WIDTH-1:0] i_qp_fatal_wqe_proc,
    input  wire                          i_qp_fatal_wqe_proc_vld,
    output wire                          o_qp_fatal_wqe_proc_rdy,

    // Response Handler interface -2 
    input  wire [C_QP_INDX_WIDTH-1:0] i_qp_fatal_rsp_hndl,
    input  wire                       i_qp_fatal_rsp_hndl_vld,
    output wire                       o_qp_fatal_rsp_hndl_rdy,

    // WQE Processor interface -2
    // 接收外来请求后，需要发送响应包
    output wire [C_QP_INDX_WIDTH-1:0] o_ob_rsp_qpid,           // 目的 QPID 是响应数据包的目标，提取的是请求包 BTH 中的目的 QPID-1
    output wire [               23:0] o_ob_rsp_psn,            // BTH.PSN to embed in response packet         
    output wire [                7:0] o_ob_rsp_aeth_syndrome,  // AETH.Syndrome to embed in response packet
    output wire [               23:0] o_ob_rsp_aeth_msn,       // AETH.MSN to embed in response packet
    output wire                       o_ob_rsp_expl_ack,
    output wire                       o_ob_rsp_vld,            // Indicates the response signals are valid     
    input  wire                       ob_rsp_fifo_full,        // Indicates the response FIFO is full 

    //Response Handler Interface -3
    output wire [                1:0] o_req_indicator,               // 0=ACK, 1=READ_REQ, 2=DUPLICATE READ REQ
    output wire [               63:0] o_physical_addr,               // Phy address for Virtual address received in READ REQ
    output wire [               31:0] o_dma_len,                     // DMA length in read req  
    output wire [               12:0] o_rd_req_pmtu,                 // PMTU of read req  
    output wire [               23:0] o_rsp_last_psn,                // last PSN of read req  
    //input  wire  [C_NUM_QP-1:0]                 i_rd_req_fifo_full,
    input  wire [C_QP_INDX_WIDTH-1:0] i_rd_req_fifo_full_set_idx,
    input  wire                       i_rd_req_fifo_full_set,
    input  wire [C_QP_INDX_WIDTH-1:0] i_rd_req_fifo_full_reset_idx,
    input  wire                       i_rd_req_fifo_full_reset,

    //PD Configuration for Intiator
    input wire [23:0] i_pd_pd_num,
    input wire [31:0] i_pd_virt_addr_0,
    input wire [31:0] i_pd_virt_addr_1,
    input wire [31:0] i_pd_buf_base_addr_0,
    input wire [31:0] i_pd_buf_base_addr_1,
    input wire [ 7:0] i_pd_buf_r_key,
    input wire [47:0] i_pd_w_r_buf_len,      // 写/读缓冲区长度(WRRDBUFLEN寄存器)
    input wire [ 3:0] i_pd_access_type,

    // QP configuration
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_req_conf_idx,  // qp1就是0
    output wire [C_PD_INDX_WIDTH-1:0] o_pd_req_conf_idx,

    input  wire [31:0] i_qp_req_conf,
    input  wire        qp_req_conf_vld,
    output wire        qp_req_conf_en,
    output wire        pd_req_conf_en,

    input  wire [C_M_AXI_ADDR_WIDTH-9:0] i_rq_buf_ba,
    output wire                          rq_buf_ba_en,

    input  wire [C_M_AXI_ADDR_WIDTH-9:0] i_rq_buf_ca,
    output wire [C_M_AXI_ADDR_WIDTH-9:0] o_rq_buf_ca,
    output wire                          rq_buf_ca_en,
    output wire                          rq_buf_ca_we,

    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_rq_wrptr_db_addr,
    output wire                          rq_wrptr_db_addr_en,

    input  wire [15:0] i_q_depth,
    output wire        q_depth_en,

    input  wire [15:0] i_rq_ci_db,
    output wire        rq_ci_db_en,

    input  wire [31:0] i_last_rq_req,
    output wire [31:0] o_last_rq_req,
    output wire        last_rq_req_en,
    output wire        last_rq_req_we,

    input  wire [23:0] i_stat_qp_msn,
    output wire [23:0] o_stat_qp_msn,
    output wire        stat_qp_msn_en,
    output wire        stat_qp_msn_we,

    input  wire [47:0] i_qp_req_remote_qp_mac_addr,
    output wire        qp_req_remote_qp_mac_addr_en,

    input  wire [127:0] i_qp_req_remote_qp_ip_addr,
    output wire         qp_req_remote_qp_ip_addr_en,

    input  wire [15:0] i_rq_wrptr_db,
    output wire [15:0] o_rq_wrptr_db,
    output wire        rq_wrptr_db_en,
    output wire        rq_wrptr_db_we,

    input  wire [23:0] i_qp_pd_num,
    output wire        qp_pd_num_en,

    output wire [C_QP_INDX_WIDTH-1:0] o_qp_rsp_conf_idx,

    input  wire [31:0] i_qp_rsp_conf,
    input  wire        qp_rsp_conf_vld,
    output wire        qp_rsp_conf_en,

    input  wire [9:0] i_stat_nak,
    output wire       stat_nak_en,
    output wire       stat_nak_we,
    output wire [9:0] o_stat_nak,

    input  wire [23:0] i_stat_resp_psn,
    output wire        stat_resp_psn_en,
    output wire        stat_resp_psn_we,
    output wire [23:0] o_stat_resp_psn,

    input  wire [23:0] i_sq_psn,
    output wire        sq_psn_en,

    input  wire [47:0] i_qp_rsp_remote_qp_mac_addr,
    output wire        qp_rsp_remote_qp_mac_addr_en,

    input  wire [127:0] i_qp_rsp_remote_qp_ip_addr,
    output wire         qp_rsp_remote_qp_ip_addr_en,

    input wire [31:0] i_qp_timeout,

    output reg o_qp_fatal_timer_idle,

    // AXI interface to flush SEND packets -- 刷新 SEND 数据包的 AXI 接口
    // rx_req_axi_mstr模块 传出的 axi接口
    output wire [                     0:0] rx_req_axi_awid,
    output wire [  C_M_AXI_ADDR_WIDTH-1:0] rx_req_axi_awaddr,
    output wire [C_M_AXI_AWUSER_WIDTH-1:0] rx_req_axi_awuser,
    output wire [                     7:0] rx_req_axi_awlen,
    output wire [                     2:0] rx_req_axi_awsize,
    output wire [                     1:0] rx_req_axi_awburst,
    output wire [                     3:0] rx_req_axi_awcache,
    output wire [                     2:0] rx_req_axi_awprot,
    output wire                            rx_req_axi_awvalid,
    input  wire                            rx_req_axi_awready,
    output wire [                   511:0] rx_req_axi_wdata,
    output wire [                    63:0] rx_req_axi_wstrb,
    output wire                            rx_req_axi_wlast,
    output wire                            rx_req_axi_wvalid,
    input  wire                            rx_req_axi_wready,
    output wire                            rx_req_axi_awlock,
    input  wire [                     0:0] rx_req_axi_bid,
    input  wire [                     1:0] rx_req_axi_bresp,
    input  wire                            rx_req_axi_bvalid,
    output wire                            rx_req_axi_bready,
    output wire [                     0:0] rx_req_axi_arid,
    output wire [  C_M_AXI_ADDR_WIDTH-1:0] rx_req_axi_araddr,
    output wire [                     7:0] rx_req_axi_arlen,
    output wire [                     2:0] rx_req_axi_arsize,
    output wire [                     1:0] rx_req_axi_arburst,
    output wire [                     3:0] rx_req_axi_arcache,
    output wire [                     2:0] rx_req_axi_arprot,
    output wire                            rx_req_axi_arvalid,
    input  wire                            rx_req_axi_arready,
    input  wire [                     0:0] rx_req_axi_rid,
    input  wire [                   511:0] rx_req_axi_rdata,
    input  wire [                     1:0] rx_req_axi_rresp,
    input  wire                            rx_req_axi_rlast,
    input  wire                            rx_req_axi_rvalid,
    output wire                            rx_req_axi_rready,
    output wire                            rx_req_axi_arlock,

    // AXI interface to flush READ Responses to SSD
    // also error packets to DDR
    output wire [                   0:0] rx_rsp_axi_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] rx_rsp_axi_awaddr,
    output wire [                   7:0] rx_rsp_axi_awlen,
    output wire [                   2:0] rx_rsp_axi_awsize,
    output wire [                   1:0] rx_rsp_axi_awburst,
    output wire [                   3:0] rx_rsp_axi_awcache,
    output wire [                   2:0] rx_rsp_axi_awprot,
    output wire                          rx_rsp_axi_awvalid,
    input  wire                          rx_rsp_axi_awready,
    output wire [                 511:0] rx_rsp_axi_wdata,
    output wire [                  63:0] rx_rsp_axi_wstrb,
    output wire                          rx_rsp_axi_wlast,
    output wire                          rx_rsp_axi_wvalid,
    input  wire                          rx_rsp_axi_wready,
    output wire                          rx_rsp_axi_awlock,
    input  wire [                   0:0] rx_rsp_axi_bid,
    input  wire [                   1:0] rx_rsp_axi_bresp,
    input  wire                          rx_rsp_axi_bvalid,
    output wire                          rx_rsp_axi_bready,
    output wire [                   0:0] rx_rsp_axi_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] rx_rsp_axi_araddr,
    output wire [                   7:0] rx_rsp_axi_arlen,
    output wire [                   2:0] rx_rsp_axi_arsize,
    output wire [                   1:0] rx_rsp_axi_arburst,
    output wire [                   3:0] rx_rsp_axi_arcache,
    output wire [                   2:0] rx_rsp_axi_arprot,
    output wire                          rx_rsp_axi_arvalid,
    input  wire                          rx_rsp_axi_arready,
    input  wire [                   0:0] rx_rsp_axi_rid,
    input  wire [                 511:0] rx_rsp_axi_rdata,
    input  wire [                   1:0] rx_rsp_axi_rresp,
    input  wire                          rx_rsp_axi_rlast,
    input  wire                          rx_rsp_axi_rvalid,
    output wire                          rx_rsp_axi_rready,
    output wire                          rx_rsp_axi_arlock,

    output wire                       waiting_for_rq_update,
    output wire [C_QP_INDX_WIDTH-1:0] waiting_for_rq_update_on_qp_num,
    // Debug signals
    output wire [               31:0] o_all_rd_req_cnt,
    output wire [               31:0] o_good_rd_req_cnt,
    output wire [               31:0] o_dup_rd_req_cnt,
    input  wire [               31:0] debug_ctrl_in,
    output reg  [               31:0] debug_sts_out,

    output wire        ieth_immdt_axis_tvalid,
    output wire        ieth_immdt_axis_tlast,
    output wire [63:0] ieth_immdt_axis_tdata,
    input  wire        ieth_immdt_axis_trdy,

    input  wire                       i_global_dbg_cnt_en,
    input  wire                       i_global_dbg_cnt_clr,
    output wire                       tx_pause_req_on,
    input  wire                       init_start,
    input  wire [C_QP_INDX_WIDTH-1:0] init_count

);

    // Local parameters
    // 数据位宽均为 512 bits = 64 Bytes, 即每个单元: 512位 = 64字节
    // 地址位宽计算空间: 总容量 = 2^地址位宽 * 数据位宽/8
    // 例: REQ_BUF总容量 = 2^9 * 64 = 2^5 KBytes = 32 KBytes
    localparam C_REQ_BUF_ADDR_WIDTH = 9;  // 32 KBytes, approx. 256 SEND packets(32 KB，约 256 个发送数据包)
    localparam C_RSP_BUF_ADDR_WIDTH = 10;  // 64 KBytes
    localparam C_HDR_BUF_ADDR_WIDTH = 9;  // 32 KBytes, 256 packets 
    localparam C_CFG_MEM_ADDR_WIDTH = C_HDR_BUF_ADDR_WIDTH - 1;  // 32 KBytes, 256 packets；8
    localparam PKT_DESC_FIFO_DATA_WIDTH = C_SIM_DEBUG ? (C_RSP_BUF_ADDR_WIDTH + 49) : (C_RSP_BUF_ADDR_WIDTH + 17);  // 27
    localparam REQ_BUF_MEM_SZ_IN_BITS = (2 ** C_REQ_BUF_ADDR_WIDTH) * 512;
    localparam RSP_BUF_MEM_SZ_IN_BITS = (2 ** C_RSP_BUF_ADDR_WIDTH) * 512;
    localparam HDR_BUF_MEM_SZ_IN_BITS = (2 ** C_HDR_BUF_ADDR_WIDTH) * 512;
    localparam HDR_BUF_QPID_MEM_SZ_IN_BITS = (2 ** C_CFG_MEM_ADDR_WIDTH) * 32;
    localparam HDR_BUF_RKEY_MEM_SZ_IN_BITS = (2 ** C_CFG_MEM_ADDR_WIDTH) * 32;

    wire                                core_rst;

    wire                                incr_inv_req_pkt_cnt;
    wire                                incr_inv_rsp_pkt_cnt;

    wire [         C_QP_INDX_WIDTH-1:0] qp_req_fatal_idx;
    wire                                set_qp_req_fatal;
    wire [         C_QP_INDX_WIDTH-1:0] qp_rsp_fatal_idx;
    wire                                set_qp_rsp_fatal;

    //wire [C_NUM_QP-1:0] qp_req_fatal_bitmap;
    //wire [C_NUM_QP-1:0] qp_rsp_fatal_bitmap;
    //reg  [C_NUM_QP-1:0] qp_async_fatal_bitmap;
    //wire [C_NUM_QP-1:0] qp_wqe_proc_fatal_bitmap;
    //wire [C_NUM_QP-1:0] qp_rsp_hndl_fatal_bitmap;
    //reg                 qp_wqe_proc_fatal_vld;
    //reg                 qp_rsp_hndl_fatal_vld;

    //reg [C_NUM_QP-1:0] qp_fatal;
    reg                                 set_qp_req_fatal_r;
    reg                                 set_qp_rsp_fatal_r;
    reg                                 clr_qp_fatal_r;
    reg  [         C_QP_INDX_WIDTH-1:0] req_fatal_idx_r;
    reg  [         C_QP_INDX_WIDTH-1:0] rsp_fatal_idx_r;
    reg  [         C_QP_INDX_WIDTH-1:0] clr_qp_fatal_idx_r;


    wire [                        15:0] rq_db_data;
    wire [                        12:0] rq_db_addr;
    wire                                rq_db_data_valid;
    wire                                rq_db_data_ack;

    reg                                 con_io_db_vld;

    wire                                set_req_pkt_valdn_err_intr;
    wire                                set_rsp_pkt_valdn_err_intr;
    wire                                set_pkt_valdn_err_intr;
    wire                                set_mad_pkt_rcvd_intr;
    wire                                set_bypass_pkt_rcvd_intr;
    wire                                set_rnr_nack_gen_intr;
    wire                                set_fatal_err_intr;
    wire [         C_QP_INDX_WIDTH-1:0] pkt_rcvd_qpid_1;
    wire                                set_qp_pkt_rcvd_intr_1;
    wire [         C_QP_INDX_WIDTH-1:0] pkt_rcvd_qpid_2;
    wire                                set_qp_pkt_rcvd_intr_2;

    wire [                        31:0] req_pkt_id;
    wire [         C_QP_INDX_WIDTH-1:0] req_qpid;
    wire [                         3:0] req_pkt_type;
    wire                                req_pkt_ip_ver;
    wire [                        15:0] req_rq_pi_db;
    wire [      C_M_AXI_ADDR_WIDTH-1:0] req_dest_addr;
    wire [    C_REQ_BUF_ADDR_WIDTH-1:0] req_buf_addr_offset;
    wire [                        12:0] req_pkt_len;
    wire [                        43:0] req_imm_data;
    wire                                req_tnfr_vld;
    wire                                rdy_to_req_tnfr;
    wire                                req_ieth_present;
    wire [                        63:0] ieth_immdt_fifo_data;
    wire                                ieth_immdt_fifo_wr_en;
    wire                                req_immdt_present;
    wire                                o_req_ieth_present;
    wire                                o_req_immdt_present;

    wire [                        31:0] o_req_pkt_id;
    wire [                         3:0] o_req_pkt_type;
    wire                                o_req_pkt_ip_ver;
    wire [                        15:0] o_req_rq_pi_db;
    wire [         C_QP_INDX_WIDTH-1:0] o_req_qpid;
    wire [      C_M_AXI_ADDR_WIDTH-1:0] o_req_dest_addr;
    wire [    C_REQ_BUF_ADDR_WIDTH-1:0] o_req_buf_addr_offset;
    wire [                        12:0] o_req_pkt_len;
    wire [                        43:0] o_req_imm_data;
    wire                                o_req_tnfr_vld;
    wire                                i_rdy_to_req_tnfr;

    wire [                        31:0] o_rsp_pkt_id;
    wire [                         2:0] o_rsp_pkt_type;
    wire [      C_M_AXI_ADDR_WIDTH-1:0] o_rsp_dest_addr;
    wire [    C_RSP_BUF_ADDR_WIDTH-1:0] o_rsp_buf_addr_offset;
    wire [                        12:0] o_rsp_pkt_len;
    wire [                        31:0] o_rsp_imm_data;
    wire                                o_rsp_tnfr_vld;
    wire                                i_rdy_to_rsp_tnfr;

    wire [                        31:0] o_req_err_pkt_id;
    wire [                         3:0] o_req_err_pkt_type;
    wire [                        31:0] o_req_err_imm_data;
    wire                                o_req_err_tnfr_vld;
    wire                                i_rdy_to_req_err_tnfr;

    wire                                req_err_tnfr_vld;
    wire                                rdy_to_req_err_tnfr;

    wire [                        31:0] rsp_pkt_id;
    wire [                         2:0] rsp_pkt_type;
    wire [      C_M_AXI_ADDR_WIDTH-1:0] rsp_dest_addr;
    wire [    C_RSP_BUF_ADDR_WIDTH-1:0] rsp_buf_addr_offset;
    wire [                        12:0] rsp_pkt_len;
    wire [                        31:0] rsp_imm_data;
    wire                                rsp_tnfr_vld;
    wire                                rdy_to_rsp_tnfr;

    wire [    C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_addra;
    wire                                req_pkt_buf_ena;
    wire [                        63:0] req_pkt_buf_wea;
    wire [                       511:0] req_pkt_buf_dina;
    wire [    C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_addrb;
    wire                                req_pkt_buf_enb;
    wire [                       511:0] req_pkt_buf_doutb;

    wire [    C_HDR_BUF_ADDR_WIDTH-1:0] req_hdr_buf_addra;
    wire                                req_hdr_buf_ena;
    wire [                        63:0] req_hdr_buf_wea;
    wire [                       511:0] req_hdr_buf_dina;
    wire [    C_HDR_BUF_ADDR_WIDTH-1:0] req_hdr_buf_addrb;
    wire                                req_hdr_buf_enb;
    wire [                       511:0] req_hdr_buf_doutb;


    wire [                         3:0] req_hdr_qpid_buf_wea;
    wire [                        23:0] req_hdr_qpid_buf_dina;
    wire [                         7:0] bth_opcode;
    wire [                        31:0] req_hdr_qpid_buf_doutb;

    wire                                req_hdr_rkey_buf_ena;
    wire [                         3:0] req_hdr_rkey_buf_wea;
    wire [                        31:0] req_hdr_rkey_buf_dina;
    wire [                        31:0] req_hdr_rkey_buf_doutb;

    wire [                        15:0] ether_type_ipv4;
    reg                                 ipv6_valid;
    reg  [                        15:0] ipv4_rkey_lsb;

    wire [    C_RSP_BUF_ADDR_WIDTH-1:0] rsp_pkt_buf_addra;
    wire                                rsp_pkt_buf_ena;
    wire [                        63:0] rsp_pkt_buf_wea;
    wire [                       511:0] rsp_pkt_buf_dina;
    wire [    C_RSP_BUF_ADDR_WIDTH-1:0] rsp_pkt_buf_addrb;
    reg  [    C_RSP_BUF_ADDR_WIDTH-1:0] rsp_pkt_buf_addrb_with_vld;
    wire                                rsp_pkt_buf_enb;
    wire [                       511:0] rsp_pkt_buf_doutb;

    wire [    C_HDR_BUF_ADDR_WIDTH-1:0] rsp_hdr_buf_addra;
    wire                                rsp_hdr_buf_ena;
    wire [                        63:0] rsp_hdr_buf_wea;
    wire [                       511:0] rsp_hdr_buf_dina;
    wire [    C_HDR_BUF_ADDR_WIDTH-1:0] rsp_hdr_buf_addrb;
    wire                                rsp_hdr_buf_enb;
    wire [                       511:0] rsp_hdr_buf_doutb;

    reg                                 rsp_ipv6_valid;
    wire [                        15:0] rsp_ether_type_ipv4;
    wire [                         3:0] rsp_hdr_qpid_buf_wea;
    wire [                        23:0] rsp_hdr_qpid_buf_dina;
    wire [                        31:0] rsp_hdr_qpid_buf_doutb;

    // req 包描述符 fifo 接口
    wire [PKT_DESC_FIFO_DATA_WIDTH-1:0] req_pkt_desc_fifo_dout;
    //wire req_pkt_desc_fifo_empty;
    reg                                 req_pkt_desc_fifo_empty_r1;
    wire                                rd_req_pkt_desc_fifo;

    // rsp 包描述符 fifo 接口
    wire [PKT_DESC_FIFO_DATA_WIDTH-1:0] rsp_pkt_desc_fifo_dout;
    wire                                rsp_pkt_desc_fifo_empty;
    wire                                rd_rsp_pkt_desc_fifo;

    wire [      C_REQ_BUF_ADDR_WIDTH:0] req_pkt_buf_sz;  // req_pkt_buf 总单元数
    wire [      C_RSP_BUF_ADDR_WIDTH:0] rsp_pkt_buf_sz;  // rsp_pkt_buf 总单元数
    wire [      C_HDR_BUF_ADDR_WIDTH:0] rsp_hdr_buf_sz;  // rsp_hdr_buf 总单元数
    wire [      C_HDR_BUF_ADDR_WIDTH:0] req_hdr_buf_sz;  // req_hdr_buf 总单元数

    wire [      C_RSP_BUF_ADDR_WIDTH:0] rsp_pkt_buf_sz_30_percnt;

    reg                                 req_pkt_buf_almost_full;
    reg                                 rsp_pkt_buf_almost_full;

    reg                                 rsp_pkt_buf_70_percnt_full;

    reg  [                         5:0] xrnic_en_reg;
    wire                                xrnic_en_pulse;

    reg  [                        31:0] free_run_cnt;

    wire                                pkt_discarded;

    reg                                 pkt_valdn_err_sticky;

    wire [                        31:0] debug_sts_reg                                                                    [0:12];

    reg  [                        15:0] o_rx_rsp_axi_wvalid_cnt;
    reg  [                        15:0] o_rx_rsp_axi_wchnl_wait_st_cnt;

    wire                                axi_awchnl_fifo_empty;
    wire                                axi_wchnl_fifo_empty;
    wire                                post_axi_tnfr_fifo_empty;

    wire                                load_qp_fatal_timer;
    reg                                 load_qp_fatal_timer_d;
    reg                                 load_qp_fatal_timer_2d;
    wire                                qp_fatal_timer_idle;
    wire                                clr_qp_fatal_timer;

    reg  [                         7:0] rsp_fifos_empty_sh_reg;
    wire                                rsp_fifos_empty;

    reg  [                         9:0] qp_fatal_timer;

    wire                                ack_coalsc_tvalid;
    wire [                       511:0] ack_coalsc_tdata;
    wire [                        63:0] ack_coalsc_tkeep;
    wire                                ack_coalsc_tlast;
    wire [                         0:0] ack_coalsc_tuser;

    wire [                C_NUM_QP-1:0] rnr_window_flag_reg;

    wire                                rx_req_ecn_update_flg;
    //wire [C_NUM_QP-1:0] rx_req_ecn_vect;
    wire [         C_QP_INDX_WIDTH-1:0] rx_req_ecn_vect_idx;
    wire                                rx_rsp_ecn_update_flg;
    //wire [C_NUM_QP-1:0] rx_rsp_ecn_vect;
    wire [         C_QP_INDX_WIDTH-1:0] rx_rsp_ecn_vect_idx;

    reg  [                        18:0] cnp_timeout_reg;

    wire                                set_cnp_sched_intr;
    reg                                 cnp_sched_pend;

    wire                                cnp_pkt_rcvd;
    wire [                         1:0] cnp_pkt_rcvd_combo;
    wire [         C_QP_INDX_WIDTH-1:0] cnp_pkt_qpid;
    reg  [                        18:0] cnp_tmr_reg;
    wire                                cnp_tmr_expired;  // 由于 i_cnp_timeout_reg_2 Reserved，这个信号一直为0
    reg                                 cnp_tmr_fifo_wr_en;
    wire                                cnp_tmr_fifo_empty;
    reg  [           C_QP_INDX_WIDTH:0] cnp_tmr_fifo_din;
    wire [           C_QP_INDX_WIDTH:0] cnp_tmr_fifo_dout;
    reg                                 cnp_tmr_fifo_rd_en;
    reg                                 cnp_phase;
    reg                                 cnp_qp_reset;
    reg                                 resp_pkt_buf_pause_on;
    reg                                 resp_hdr_buf_pause_on;
    reg                                 req_pkt_buf_pause_on;
    reg                                 req_hdr_buf_pause_on;
    reg                                 tx_pause_req_on_d;
    wire                                tx_pause_req_rising_edge;
    reg  [                         4:0] assert_cnt;
    reg                                 cnt_en;
    reg  [           C_QP_INDX_WIDTH:0] qp_under_recovery_cnt;

    //reg  [31:0]                cnp_rx_pkt_cnt;

    // Other miscellanous logic & internal signals
    assign core_rst = ~core_rst_n;

    //wire clr_debug_sts = (debug_ctrl_in[31] || (C_EN_DEBUG_PORTS == 0)) ? 1'b1 : 1'b0;
    wire clr_debug_sts = debug_ctrl_in[31];

    always @(posedge core_clk)
        if (core_rst) o_inc_inv_pkt_cnt <= 16'd0;
        else o_inc_inv_pkt_cnt <= o_inc_inv_pkt_cnt + incr_inv_req_pkt_cnt + incr_inv_rsp_pkt_cnt;

    //assign qp_req_fatal_bitmap = {{(C_NUM_QP-1){1'b0}}, set_qp_req_fatal} << qp_req_fatal_idx;
    //assign qp_rsp_fatal_bitmap = {{(C_NUM_QP-1){1'b0}}, set_qp_rsp_fatal} << qp_rsp_fatal_idx;

    //assign qp_wqe_proc_fatal_bitmap = {{(C_NUM_QP-1){1'b0}}, i_qp_fatal_wqe_proc_vld} << i_qp_fatal_wqe_proc;
    //assign qp_rsp_hndl_fatal_bitmap = {{(C_NUM_QP-1){1'b0}}, i_qp_fatal_rsp_hndl_vld} << i_qp_fatal_rsp_hndl;

    assign tx_pause_req_on = (resp_pkt_buf_pause_on | resp_hdr_buf_pause_on | req_pkt_buf_pause_on | req_hdr_buf_pause_on);

    //always @ (posedge core_clk)
    //begin
    //  if (core_rst)
    //  begin
    //    qp_async_fatal_bitmap <= 0;
    //    qp_wqe_proc_fatal_vld <= 0;
    //    qp_rsp_hndl_fatal_vld <= 0;
    //  end
    //  else
    //  begin
    //    qp_async_fatal_bitmap <= qp_wqe_proc_fatal_bitmap | qp_rsp_hndl_fatal_bitmap;
    //    qp_wqe_proc_fatal_vld <= i_qp_fatal_wqe_proc_vld;
    //    qp_rsp_hndl_fatal_vld <= i_qp_fatal_rsp_hndl_vld;
    //  end
    //end

    //////////////////////////////////////////////////////////////////////////////////////////////
    //assumption here is that, set_qp_req_fatal, set_qp_rsp_fatal, and
    //clr_qp_fatal signals will be asserted only for single clock cycle, and
    //these signals will be asserted with a gap of minimum of 4 clock cycles
    //////////////////////////////////////////////////////////////////////////////////////////////

    always @(posedge core_clk) begin
        if (core_rst) begin
            set_qp_req_fatal_r <= 1'b0;
            set_qp_rsp_fatal_r <= 1'b0;
            clr_qp_fatal_r     <= 1'b0;
            req_fatal_idx_r    <= 'd0;
            rsp_fatal_idx_r    <= 'd0;
        end else begin
            set_qp_req_fatal_r <= set_qp_req_fatal;
            req_fatal_idx_r    <= qp_req_fatal_idx;
            if (~(set_qp_req_fatal_r && set_qp_rsp_fatal_r)) begin
                set_qp_rsp_fatal_r <= set_qp_rsp_fatal;
                rsp_fatal_idx_r    <= qp_rsp_fatal_idx;
            end
            if (~((set_qp_req_fatal_r || set_qp_rsp_fatal_r) && clr_qp_fatal_r)) begin
                clr_qp_fatal_r     <= i_clr_qp_fatal;
                clr_qp_fatal_idx_r <= i_clr_qp_fatal_idx;
            end
        end
    end

    assign o_qp_fatal_wr_en   = set_qp_req_fatal_r || set_qp_rsp_fatal_r || clr_qp_fatal_r;
    assign o_qp_fatal_wr_idx  = set_qp_req_fatal_r ? req_fatal_idx_r : (set_qp_rsp_fatal_r ? rsp_fatal_idx_r : clr_qp_fatal_idx_r);
    assign o_qp_fatal_wr_data = (set_qp_req_fatal_r || set_qp_rsp_fatal_r) ? 1'b1 : 1'b0;

    //always @(posedge core_clk)
    //  if (core_rst)
    //    qp_fatal <= {(C_NUM_QP){1'b0}};
    //  else if (|{i_clr_qp_fatal, set_qp_req_fatal, set_qp_rsp_fatal}) //, qp_wqe_proc_fatal_vld})//,qp_rsp_hndl_fatal_vld })
    //    qp_fatal <= (qp_fatal & ~i_clr_qp_fatal) | qp_req_fatal_bitmap | qp_rsp_fatal_bitmap;// | qp_async_fatal_bitmap;

    //assign o_qp_fatal = qp_fatal;

    //always @(posedge core_clk)
    //  if (core_rst)
    //    o_cnp_sched_reg <= {(C_NUM_QP){1'b0}};
    //  else if (|{i_clr_cnp_sched | rx_req_ecn_update_flg | rx_rsp_ecn_update_flg})
    //    o_cnp_sched_reg <= (o_cnp_sched_reg & ~i_clr_cnp_sched) | ({(C_NUM_QP){rx_req_ecn_update_flg}} & rx_req_ecn_vect) |
    //                     ({(C_NUM_QP){rx_rsp_ecn_update_flg}} & rx_rsp_ecn_vect);
    reg                       rx_rsp_ecn_update_flg_r;
    reg                       rx_req_ecn_update_flg_r;
    reg [C_QP_INDX_WIDTH-1:0] rx_rsp_ecn_vect_idx_r;
    reg [C_QP_INDX_WIDTH-1:0] rx_req_ecn_vect_idx_r;
    reg [               31:0] clr_cnp_sched_r;
    reg [C_QP_INDX_WIDTH-1:0] clr_cnp_sched_wr_addr_r;
    genvar i;

    always @(*) begin
        if (rx_req_ecn_update_flg_r) begin
            o_cnp_sched_wr_addr = rx_req_ecn_vect_idx_r[C_QP_INDX_WIDTH-1:0];
        end else if (rx_rsp_ecn_update_flg_r) begin
            o_cnp_sched_wr_addr = rx_rsp_ecn_vect_idx_r[C_QP_INDX_WIDTH-1:0];
        end else begin
            o_cnp_sched_wr_addr = clr_cnp_sched_wr_addr_r;
        end
    end

    // CNP 分组计划中断状态（按位计算）
    // o_cnp_sched_wr_data存放在 CNPSCHDSTS1~64REG寄存器中
    generate
        if (C_NUM_QP < 32) begin
            for (i = 0; i < 32; i = i + 1) begin
                always @(*) begin
                    if (i >= C_NUM_QP) begin
                        o_cnp_sched_wr_data[i] = 1'b0;
                        o_cnp_sched_wr_en[i]   = 1'b0;
                    end else if (rx_req_ecn_update_flg_r) begin
                        if (rx_req_ecn_vect_idx_r[C_QP_INDX_WIDTH-1:0] == i[C_QP_INDX_WIDTH-1:0]) begin
                            o_cnp_sched_wr_data[i] = 1'b1;
                            o_cnp_sched_wr_en[i]   = 1'b1;
                        end else begin
                            o_cnp_sched_wr_data[i] = 1'b0;
                            o_cnp_sched_wr_en[i]   = 1'b0;
                        end
                    end else if (rx_rsp_ecn_update_flg_r) begin
                        if (rx_rsp_ecn_vect_idx_r[C_QP_INDX_WIDTH-1:0] == i[C_QP_INDX_WIDTH-1:0]) begin
                            o_cnp_sched_wr_data[i] = 1'b1;
                            o_cnp_sched_wr_en[i]   = 1'b1;
                        end else begin
                            o_cnp_sched_wr_data[i] = 1'b0;
                            o_cnp_sched_wr_en[i]   = 1'b0;
                        end
                    end else begin
                        o_cnp_sched_wr_data[i] = 1'b0;
                        o_cnp_sched_wr_en[i]   = clr_cnp_sched_r[i];  // 保持不变
                    end
                end
            end
        end else begin
            for (i = 0; i < 32; i = i + 1) begin
                always @(*) begin
                    if (rx_req_ecn_update_flg_r) begin
                        if (rx_req_ecn_vect_idx_r[4:0] == i[4:0]) begin
                            o_cnp_sched_wr_data[i] = 1'b1;
                            o_cnp_sched_wr_en[i]   = 1'b1;
                        end else begin
                            o_cnp_sched_wr_data[i] = 1'b0;
                            o_cnp_sched_wr_en[i]   = 1'b0;
                        end
                    end else if (rx_rsp_ecn_update_flg_r) begin
                        if (rx_rsp_ecn_vect_idx_r[4:0] == i[4:0]) begin
                            o_cnp_sched_wr_data[i] = 1'b1;
                            o_cnp_sched_wr_en[i]   = 1'b1;
                        end else begin
                            o_cnp_sched_wr_data[i] = 1'b0;
                            o_cnp_sched_wr_en[i]   = 1'b0;
                        end
                    end else begin
                        o_cnp_sched_wr_data[i] = 1'b0;
                        o_cnp_sched_wr_en[i]   = clr_cnp_sched_r[i];  // 保持不变
                    end
                end
            end
        end
    endgenerate

    // 
    always @(posedge core_clk) begin
        if (core_rst) begin
            rx_rsp_ecn_update_flg_r <= 1'b0;
            rx_req_ecn_update_flg_r <= 1'b0;
            rx_rsp_ecn_vect_idx_r   <= 'd0;
            rx_req_ecn_vect_idx_r   <= 'd0;
            clr_cnp_sched_r         <= 32'd0;
            clr_cnp_sched_wr_addr_r <= 32'd0;
            cnp_sched_pend          <= 1'b0;
        end else begin
            // 只要有 ECN 更新挂起，就置 cnp_sched_pend
            if (rx_req_ecn_update_flg_r || rx_rsp_ecn_update_flg_r) begin
                cnp_sched_pend <= 1'b1;
            end else if (|clr_cnp_sched_r) begin
                cnp_sched_pend <= 1'b0;
            end

            rx_req_ecn_update_flg_r <= rx_req_ecn_update_flg;
            rx_req_ecn_vect_idx_r   <= rx_req_ecn_vect_idx;

            // req和rsp ecn同时更新
            if (~(rx_req_ecn_update_flg_r && rx_rsp_ecn_update_flg_r)) begin
                rx_rsp_ecn_update_flg_r <= rx_rsp_ecn_update_flg;
                rx_rsp_ecn_vect_idx_r   <= rx_rsp_ecn_vect_idx;
            end

            // req和rsp ecn没更新 & CNP 计划中断寄存器(CNPSCHDSTS1~64REG)值也为0时，保持不变
            if (~((rx_req_ecn_update_flg_r || rx_rsp_ecn_update_flg_r) && |clr_cnp_sched_r)) begin
                clr_cnp_sched_r         <= i_clr_cnp_sched;
                clr_cnp_sched_wr_addr_r <= i_clr_cnp_sched_wr_addr;
            end
        end
    end

    // 周期计数器 cnp_timeout_reg 到 1 且 cnp_sched_pend=1 时，产生 set_cnp_sched_intr 进入中断
    always @(posedge core_clk) begin
        if (core_rst) cnp_timeout_reg <= 18'd0;
        else if (xrnic_en_pulse) cnp_timeout_reg <= {i_cnp_timeout_reg_1, 14'h0};
        else if (xrnic_en_reg[5]) begin
            if (|cnp_timeout_reg) cnp_timeout_reg <= cnp_timeout_reg - 'd1;
            else cnp_timeout_reg <= {i_cnp_timeout_reg_1, 14'h0};
        end
    end
    //assign set_cnp_sched_intr = ((cnp_timeout_reg == 18'd1) && (o_cnp_sched_reg != 'd0)) ? 1'b1 : 1'b0;
    assign set_cnp_sched_intr = ((cnp_timeout_reg == 18'd1) && cnp_sched_pend) ? 1'b1 : 1'b0;

    always @(posedge core_clk) begin
        if (core_rst) begin
            o_cnp_rx_pkt_cnt <= 0;
        end else begin
            if (cnp_pkt_rcvd) o_cnp_rx_pkt_cnt <= o_cnp_rx_pkt_cnt + 1;
        end
    end

    // 由于 i_cnp_timeout_reg_2 Reserved，这段不会执行
    always @(posedge core_clk)
        if (core_rst) cnp_tmr_reg <= 18'd0;
        else if (cnp_pkt_rcvd) cnp_tmr_reg <= {i_cnp_timeout_reg_2, 14'h0};
        else if (|cnp_tmr_reg) cnp_tmr_reg <= cnp_tmr_reg - 'd1;
    assign cnp_tmr_expired = (cnp_tmr_reg == 18'd1) ? 1'b1 : 1'b0;

    //always @(posedge core_clk)
    //  if (core_rst)
    //    o_cnp_pkt_qp_bitmap <= {(C_NUM_QP){1'b0}};
    //  else if (cnp_pkt_rcvd)
    //    o_cnp_pkt_qp_bitmap <= o_cnp_pkt_qp_bitmap | ({{(C_NUM_QP-1){1'b0}}, 1'b1} << cnp_pkt_qpid);
    //  else if (cnp_tmr_expired)
    //    o_cnp_pkt_qp_bitmap <= {(C_NUM_QP){1'b0}};


    //==========================
    // cnp_tmr_fifo (STD), DEPTH = 512
    //========================== 
    xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE   ("auto"),                               //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           ("no_ecc"),                             //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   ((C_NUM_QP < 16) ? 16 : C_NUM_QP),      //positive integer									
        .WRITE_DATA_WIDTH   (1 + C_QP_INDX_WIDTH),                  //positive integer									
        .WR_DATA_COUNT_WIDTH(C_QP_INDX_WIDTH),                      //positive integer									
        .PROG_FULL_THRESH   ((C_NUM_QP < 16) ? 11 : C_NUM_QP - 5),  //positive integer									
        .FULL_RESET_VALUE   (0),                                    //positive integer; 0 or 1									
        .READ_MODE          ("fwft"),                               //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (1),                                    //positive integer;									
        .READ_DATA_WIDTH    (1 + C_QP_INDX_WIDTH),                  //positive integer									
        .RD_DATA_COUNT_WIDTH(C_QP_INDX_WIDTH),                      //positive integer									
        .PROG_EMPTY_THRESH  (10),                                   //positive integer									
        .DOUT_RESET_VALUE   ("0"),                                  //string									
        .WAKEUP_TIME        (0)                                     //positive integer; 0 or 2;									            
    ) cnp_tmr_fifo (
        .sleep        (1'b0),
        .rst          (core_rst),
        .wr_clk       (core_clk),
        .wr_en        (cnp_tmr_fifo_wr_en),
        .din          (cnp_tmr_fifo_din),
        .full         (),
        .prog_full    (),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (),
        .rd_en        (cnp_tmr_fifo_rd_en),
        .dout         (cnp_tmr_fifo_dout),
        .empty        (cnp_tmr_fifo_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      ()
    );

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// cnp_pkt_rcvd LUTRAM START //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(2),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_cnp_pkt_rcvd_lutram (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || o_cnp_pkt_rcvd_wr_en),
        .wr_addr (init_start ? init_count : o_cnp_pkt_rcvd_wr_idx),
        .wr_data (init_start ? 2'b00 : {cnp_phase, o_cnp_pkt_rcvd_wr_data}),
        .rd_en   (1'b1),
        .rd_addr (cnp_pkt_qpid),
        .rd_data (cnp_pkt_rcvd_combo)
    );
    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// cnp_pkt_rcvd LUTRAM END ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    always @(posedge core_clk) begin
        if (core_rst) begin
            o_cnp_pkt_rcvd_wr_en   <= 1'b0;
            o_cnp_pkt_rcvd_wr_data <= 1'b0;
            o_cnp_pkt_rcvd_wr_idx  <= 'd0;
            cnp_tmr_fifo_rd_en     <= 1'b0;
            cnp_tmr_fifo_wr_en     <= 1'b0;
            cnp_tmr_fifo_din       <= 'd0;
            cnp_phase              <= 1'b0;
            cnp_qp_reset           <= 1'b0;
        end else begin
            if (cnp_tmr_expired) begin
                cnp_phase <= ~cnp_phase;
            end

            if (cnp_tmr_expired) begin
                cnp_qp_reset <= 1'b1;
            end else if (cnp_tmr_fifo_empty || cnp_tmr_fifo_dout[C_QP_INDX_WIDTH] == cnp_phase) begin
                cnp_qp_reset <= 1'b0;
            end

            if (cnp_pkt_rcvd && (~cnp_pkt_rcvd_combo[0] || (cnp_pkt_rcvd_combo[1] ^ cnp_phase))) begin
                o_cnp_pkt_rcvd_wr_en   <= 1'b1;
                o_cnp_pkt_rcvd_wr_idx  <= cnp_pkt_qpid;
                o_cnp_pkt_rcvd_wr_data <= 1'b1;
                cnp_tmr_fifo_din       <= {cnp_phase, cnp_pkt_qpid};
                cnp_tmr_fifo_wr_en     <= 1'b1;
                cnp_tmr_fifo_rd_en     <= 1'b0;
            end else if (cnp_qp_reset && (~cnp_tmr_fifo_empty && (cnp_tmr_fifo_dout[C_QP_INDX_WIDTH] ^ cnp_phase))) begin
                o_cnp_pkt_rcvd_wr_en   <= 1'b1;
                o_cnp_pkt_rcvd_wr_idx  <= cnp_tmr_fifo_dout[C_QP_INDX_WIDTH-1:0];
                o_cnp_pkt_rcvd_wr_data <= 1'b0;
                cnp_tmr_fifo_rd_en     <= 1'b1;
                cnp_tmr_fifo_wr_en     <= 1'b0;
            end else begin
                o_cnp_pkt_rcvd_wr_en   <= 1'b0;
                o_cnp_pkt_rcvd_wr_data <= 1'b0;
                cnp_tmr_fifo_wr_en     <= 1'b0;
                cnp_tmr_fifo_rd_en     <= 1'b0;
            end
        end
    end

    /* wire qp_under_recovery_wr_idx_rd_data;
    sdp_lut_ram_wrapper #(
    	.C_RAM_WORD_WIDTH(1),
    	.C_RAM_DEPTH(C_NUM_QP),
    	.C_COMBO_BYPASS(1)
    )	inst_qp_under_recovery_cnt_or_reduce(
    	.core_clk		(core_clk),
    	.core_rst       (!core_rstn),
    	.wr_en          (init_start || qp_under_recovery_wr_en),
    	.wr_addr        (init_start ? init_count : qp_under_recovery_wr_idx),
    	.wr_data        (init_start ? 1'b0 : qp_under_recovery_wr_data),
    	.rd_en          (1'b1),
    	.rd_addr        (qp_under_recovery_wr_idx),
    	.rd_data		(qp_under_recovery_wr_idx_rd_data)
    );
                

    always @(posedge core_clk) begin
        if(core_rst) begin
            qp_under_recovery_cnt <= {C_QP_INDX_WIDTH+1{1'b0}};
        end else begin
            case ({qp_under_recovery_wr_en,qp_under_recovery_wr_data,qp_under_recovery_wr_idx_rd_data})
    			3'b110: qp_under_recovery_cnt <= qp_under_recovery_cnt + 1;
    			3'b101: qp_under_recovery_cnt <= qp_under_recovery_cnt - 1;
    			default : qp_under_recovery_cnt <= qp_under_recovery_cnt;
    		  endcase
        end
    end */

    /*   always @(posedge core_clk) begin
        if(core_rst) begin
            qp_under_recovery_cnt <= {C_QP_INDX_WIDTH+1{1'b0}};
        end else begin
            if(qp_under_recovery_wr_en && qp_under_recovery_wr_data) begin
                qp_under_recovery_cnt <= qp_under_recovery_cnt + ({{C_QP_INDX_WIDTH{1'b0}},1'b1});
            end else if(qp_under_recovery_wr_en && ~qp_under_recovery_wr_data) begin
                qp_under_recovery_cnt <= qp_under_recovery_cnt - ({{C_QP_INDX_WIDTH{1'b0}},1'b1});
            end
        end
    end*/
    //assign load_qp_fatal_timer = set_qp_req_fatal | set_qp_rsp_fatal | (|qp_under_recovery);
    assign load_qp_fatal_timer = set_qp_req_fatal | set_qp_rsp_fatal | (qp_under_recovery_wr_en && qp_under_recovery_wr_data);

    always @(posedge core_clk)
        if (core_rst) rsp_fifos_empty_sh_reg <= 8'hFF;
        else if (load_qp_fatal_timer) rsp_fifos_empty_sh_reg <= 8'h00;
        else rsp_fifos_empty_sh_reg <= {rsp_fifos_empty_sh_reg[6:0], ~rx_rsp_axi_awvalid & ~rx_rsp_axi_wvalid & axi_awchnl_fifo_empty & axi_wchnl_fifo_empty & post_axi_tnfr_fifo_empty};

    assign rsp_fifos_empty = &rsp_fifos_empty_sh_reg;

    always @(posedge core_clk)
        if (core_rst) qp_fatal_timer <= 10'd0;
        else if (load_qp_fatal_timer) qp_fatal_timer <= 10'h3FF;
        else if (rsp_fifos_empty) qp_fatal_timer <= 10'd0;
        else if ((rx_rsp_axi_bvalid && rx_rsp_axi_bready) && (qp_fatal_timer != 10'd0)) qp_fatal_timer <= qp_fatal_timer - 10'd1;

    assign qp_fatal_timer_idle = (qp_fatal_timer == 10'd0) ? 1'b1 : 1'b0;

    assign clr_qp_fatal_timer  = rsp_fifos_empty | qp_fatal_timer_idle;

    always @(posedge core_clk)
        if (core_rst) o_qp_fatal_timer_idle <= 1'b1;
        else if (load_qp_fatal_timer || clr_qp_fatal_timer) o_qp_fatal_timer_idle <= ~load_qp_fatal_timer;

    assign o_rq_db_data[31:16]      = 16'd0;

    assign set_pkt_valdn_err_intr   = set_req_pkt_valdn_err_intr | set_rsp_pkt_valdn_err_intr;

    assign req_pkt_buf_sz           = {1'b1, {(C_REQ_BUF_ADDR_WIDTH) {1'b0}}};  // 2^9 = 512个单元, 即可以包含 512 个 512bit(64B) 的数据
    assign rsp_pkt_buf_sz           = {1'b1, {(C_RSP_BUF_ADDR_WIDTH) {1'b0}}};  // 2^10 = 1024个单元
    assign req_hdr_buf_sz           = {1'b1, {(C_HDR_BUF_ADDR_WIDTH) {1'b0}}};
    assign rsp_hdr_buf_sz           = {1'b1, {(C_HDR_BUF_ADDR_WIDTH) {1'b0}}};

    assign rsp_pkt_buf_sz_30_percnt = (rsp_pkt_buf_sz >> 2) + (rsp_pkt_buf_sz >> 5);

    // 检测 req_pkt_buf 未使用空间单元是否 < 70
    always @(posedge core_clk)
        if (core_rst) req_pkt_buf_almost_full <= 1'b0;
        else if (req_pkt_buf_addra >= req_pkt_buf_addrb) begin
            if ((req_pkt_buf_sz - req_pkt_buf_addra + req_pkt_buf_addrb) < 'd70) req_pkt_buf_almost_full <= 1'b1;
            else req_pkt_buf_almost_full <= 1'b0;
        end else if (req_pkt_buf_addrb - req_pkt_buf_addra < 'd70) req_pkt_buf_almost_full <= 1'b1;
        else req_pkt_buf_almost_full <= 1'b0;

    //Pause threshold generation


    always @(posedge core_clk)
        if (core_rst) cnt_en <= 1'b0;
        else if (tx_pause_req_on) cnt_en <= 1'b1;
        else if (assert_cnt == 5'h1e) cnt_en <= 1'b0;

    always @(posedge core_clk)
        if (core_rst) assert_cnt <= 5'h0;
        else if (cnt_en || tx_pause_req_on) assert_cnt <= assert_cnt + 1;
        else if (assert_cnt == 5'h1f) assert_cnt <= assert_cnt;


    always @(posedge core_clk)
        if (core_rst) req_pkt_buf_pause_on <= 1'b0;
        else if (req_pkt_buf_addra >= req_pkt_buf_addrb) begin
            if ((req_pkt_buf_sz - req_pkt_buf_addra + req_pkt_buf_addrb) < i_xrnic_pause_on) req_pkt_buf_pause_on <= 1'b1;
            else if (((req_pkt_buf_sz - req_pkt_buf_addra + req_pkt_buf_addrb) > i_xrnic_pause_off) && assert_cnt == 5'hf) req_pkt_buf_pause_on <= 1'b0;
        end else if ((req_pkt_buf_addrb - req_pkt_buf_addra) < i_xrnic_pause_on) req_pkt_buf_pause_on <= 1'b1;
        else if (((req_pkt_buf_addrb - req_pkt_buf_addra) > i_xrnic_pause_off) && assert_cnt == 5'hf) req_pkt_buf_pause_on <= 1'b0;

    always @(posedge core_clk)
        if (core_rst) resp_pkt_buf_pause_on <= 1'b0;
        else if (rsp_pkt_buf_addra >= rsp_pkt_buf_addrb_with_vld) begin
            if ((rsp_pkt_buf_sz - rsp_pkt_buf_addra + rsp_pkt_buf_addrb_with_vld) <= i_xrnic_pause_on) resp_pkt_buf_pause_on <= 1'b1;
            else if (((rsp_pkt_buf_sz - rsp_pkt_buf_addra + rsp_pkt_buf_addrb_with_vld) >= i_xrnic_pause_off) && assert_cnt >= 5'hf) resp_pkt_buf_pause_on <= 1'b0;
        end else if (rsp_pkt_buf_addrb_with_vld - rsp_pkt_buf_addra <= i_xrnic_pause_on) resp_pkt_buf_pause_on <= 1'b1;
        else if (((rsp_pkt_buf_addrb_with_vld - rsp_pkt_buf_addra) >= i_xrnic_pause_off) && assert_cnt >= 5'hf) resp_pkt_buf_pause_on <= 1'b0;

    always @(posedge core_clk)
        if (core_rst) req_hdr_buf_pause_on <= 1'b0;
        else if (req_hdr_buf_addra >= req_hdr_buf_addrb) begin
            if ((req_hdr_buf_sz - req_hdr_buf_addra + req_hdr_buf_addrb) < i_xrnic_pause_on) req_hdr_buf_pause_on <= 1'b1;
            else if (((req_hdr_buf_sz - req_hdr_buf_addra + req_hdr_buf_addrb) > i_xrnic_pause_off) && assert_cnt == 5'hf) req_hdr_buf_pause_on <= 1'b0;
        end else if ((req_hdr_buf_addrb - req_hdr_buf_addra) < i_xrnic_pause_on) req_hdr_buf_pause_on <= 1'b1;
        else if (((req_hdr_buf_addrb - req_hdr_buf_addra) > i_xrnic_pause_off) && assert_cnt == 5'hf) req_hdr_buf_pause_on <= 1'b0;

    always @(posedge core_clk)
        if (core_rst) resp_hdr_buf_pause_on <= 1'b0;
        else if (rsp_hdr_buf_addra >= rsp_hdr_buf_addrb) begin
            if ((rsp_hdr_buf_sz - rsp_hdr_buf_addra + rsp_hdr_buf_addrb) < i_xrnic_pause_on) resp_hdr_buf_pause_on <= 1'b1;
            else if (((rsp_hdr_buf_sz - rsp_hdr_buf_addra + rsp_hdr_buf_addrb) > i_xrnic_pause_off) && assert_cnt == 5'hf) resp_hdr_buf_pause_on <= 1'b0;
        end else if ((rsp_hdr_buf_addrb - rsp_hdr_buf_addra) < i_xrnic_pause_on) resp_hdr_buf_pause_on <= 1'b1;
        else if (((rsp_hdr_buf_addrb - rsp_hdr_buf_addra) > i_xrnic_pause_off) && assert_cnt == 5'hf) resp_hdr_buf_pause_on <= 1'b0;
    /////////////////////////////////////////////////////  
    always @(posedge core_clk)
        if (core_rst) rsp_pkt_buf_addrb_with_vld <= 0;
        else if (rsp_pkt_buf_enb) rsp_pkt_buf_addrb_with_vld <= rsp_pkt_buf_addrb;  // 读使能=1时，rsp_pkt_buf_addrb_with_vld=读地址
        else rsp_pkt_buf_addrb_with_vld <= rsp_pkt_buf_addrb_with_vld;  // 读使能=0时, 保持不变

    // 检测 rsp_pkt_buf 未使用空间单元是否 < 70
    always @(posedge core_clk)
        if (core_rst) rsp_pkt_buf_almost_full <= 1'b0;
        else if (rsp_pkt_buf_addra >= rsp_pkt_buf_addrb_with_vld) begin  // 写地址 ≥ 读地址的情况
            // (rsp_pkt_buf_sz - rsp_pkt_buf_addra + rsp_pkt_buf_addrb_with_vld) 得到当前未使用的空间单元数
            // 如果未使用空间单元 < 70，标志位置1。
            if ((rsp_pkt_buf_sz - rsp_pkt_buf_addra + rsp_pkt_buf_addrb_with_vld) < 'd70) rsp_pkt_buf_almost_full <= 1'b1;
            else rsp_pkt_buf_almost_full <= 1'b0;

        end else if (rsp_pkt_buf_addrb_with_vld - rsp_pkt_buf_addra < 'd70) rsp_pkt_buf_almost_full <= 1'b1;  // 写地址-读地址 < 的情况
        else rsp_pkt_buf_almost_full <= 1'b0;

    // 同上always块功能相同，标志换为rsp_pkt_buf_70_percnt_full
    always @(posedge core_clk)
        if (core_rst) rsp_pkt_buf_70_percnt_full <= 1'b0;
        else if (rsp_pkt_buf_addra >= rsp_pkt_buf_addrb_with_vld) begin  // 写地址 ≥ 读地址的情况
            if ((rsp_pkt_buf_sz - rsp_pkt_buf_addra + rsp_pkt_buf_addrb_with_vld) < 'd70) rsp_pkt_buf_70_percnt_full <= 1'b1;
            else rsp_pkt_buf_70_percnt_full <= 1'b0;
        end else if (rsp_pkt_buf_addrb_with_vld - rsp_pkt_buf_addra < 'd70) rsp_pkt_buf_70_percnt_full <= 1'b1;
        else rsp_pkt_buf_70_percnt_full <= 1'b0;

    // 数据包先进入 ack_coalsc_filt 模块进行 ACK 包过滤，再进入 cmac_rx_intf 模块进行包分类和缓冲区管理并统计信息
    // ----------------------------------------------------------------------------------------------
    // rsp_pkt_buf 剩余空间紧张时识别 IPv4/IPv6 ACK 包并抑制其向下游传递，缓解拥塞
    ack_coalsc_filt u_ack_coalsc_filt (
        .clk            (core_clk),
        .reset          (core_rst),
        .ack_coalsc_en  (1'b1),                        // 使能ACK包过滤
        .ack_coalsc_cond(rsp_pkt_buf_70_percnt_full),  // 未使用空间单元 < 70？
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tkeep   (s_axis_tkeep),
        .s_axis_tlast   (s_axis_tlast),
        .s_axis_tuser   (s_axis_tuser),
        .m_axis_tvalid  (ack_coalsc_tvalid),
        .m_axis_tdata   (ack_coalsc_tdata),
        .m_axis_tkeep   (ack_coalsc_tkeep),
        .m_axis_tlast   (ack_coalsc_tlast),
        .m_axis_tuser   (ack_coalsc_tuser)
    );

    // xrnic_en开始一直拉高时，下下个时钟周期xrnic_en_pulse拉高，且保持4个时钟周期，随后拉低
    always @(posedge core_clk)
        if (core_rst) xrnic_en_reg <= {6{1'b0}};
        else xrnic_en_reg <= {xrnic_en_reg[5:0], i_xrnic_en};
    assign xrnic_en_pulse = ~xrnic_en_reg[5] & xrnic_en_reg[1];  // 当i_xrnic_en拉高时，下下个时钟周期xrnic_en_pulse拉高，且保持4个时钟周期

    always @(posedge core_clk)
        if (core_rst) free_run_cnt <= 32'd0;
        else free_run_cnt <= free_run_cnt + 'd1;

    // 1、包分类：解析以太网、IP、BTH等协议头 和 进行FCS、IPv4头校验和、ICRC校验 以 区分请求包和响应包
    // 2、缓冲区管理：根据包类型（req | rsp）将包头、完整包（不含FCS）、包描述符写入不同的缓冲区，若是完整包缓冲区满则丢弃该包
    // 3、统计信息：统计接收的包总数、丢弃包数、最后接收包的操作码、QP ID、PSN
    cmac_rx_intf #(
        .C_SIM_DEBUG(C_SIM_DEBUG),
        .C_NUM_QP(C_NUM_QP),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .PKT_DESC_FIFO_DATA_WIDTH(PKT_DESC_FIFO_DATA_WIDTH),
        .C_REQ_BUF_ADDR_WIDTH(C_REQ_BUF_ADDR_WIDTH),
        .C_RSP_BUF_ADDR_WIDTH(C_RSP_BUF_ADDR_WIDTH),
        .C_HDR_BUF_ADDR_WIDTH(C_HDR_BUF_ADDR_WIDTH)
    ) u_cmac_rx_intf (
        .clk          (core_clk),
        .reset        (core_rst),
        .s_axis_tvalid(ack_coalsc_tvalid),
        .s_axis_tdata (ack_coalsc_tdata),
        .s_axis_tkeep (ack_coalsc_tkeep),
        .s_axis_tlast (ack_coalsc_tlast),
        .s_axis_tuser (ack_coalsc_tuser),

        .xrnic_en (i_xrnic_en),
        .bypass_en(i_depkt_bypass_en),

        .o_inc_all_pkt_cnt   (pkt_all_cnt),
        .o_inc_drop_pkt_cnt  (pkt_disc_cnt),
        .o_last_in_pkt_opcode(o_last_in_pkt_opcode),
        .o_last_in_pkt_qpid  (o_last_in_pkt_qpid),
        .o_last_in_pkt_psn   (o_last_in_pkt_psn),

        .req_pkt_buf_almost_full(req_pkt_buf_almost_full),  // 请求包缓冲区快满标志（未使用空间单元<70？）
        .rsp_pkt_buf_almost_full(rsp_pkt_buf_almost_full),  // 响应包缓冲区快满标志（未使用空间单元<70？）

        .pkt_discarded_to_req_hv(pkt_discarded),
        .rnr_window_flag_reg    (rnr_window_flag_reg),

        .req_pkt_desc_fifo_dout(req_pkt_desc_fifo_dout),
        .rd_req_pkt_desc_fifo  (rd_req_pkt_desc_fifo),

        .rsp_pkt_desc_fifo_dout(rsp_pkt_desc_fifo_dout),
        .rd_rsp_pkt_desc_fifo  (rd_rsp_pkt_desc_fifo),

        // 包
        .req_pkt_buf_addra(req_pkt_buf_addra),
        .req_pkt_buf_ena  (req_pkt_buf_ena),
        .req_pkt_buf_wea  (req_pkt_buf_wea),
        .req_pkt_buf_dina (req_pkt_buf_dina),
        // 包头
        .req_hdr_buf_addra(req_hdr_buf_addra),
        .req_hdr_buf_ena  (req_hdr_buf_ena),
        .req_hdr_buf_wea  (req_hdr_buf_wea),
        .req_hdr_buf_dina (req_hdr_buf_dina),

        // 包
        .rsp_pkt_buf_addra(rsp_pkt_buf_addra),
        .rsp_pkt_buf_ena  (rsp_pkt_buf_ena),
        .rsp_pkt_buf_wea  (rsp_pkt_buf_wea),
        .rsp_pkt_buf_dina (rsp_pkt_buf_dina),
        // 包头
        .rsp_hdr_buf_addra(rsp_hdr_buf_addra),
        .rsp_hdr_buf_ena  (rsp_hdr_buf_ena),
        .rsp_hdr_buf_wea  (rsp_hdr_buf_wea),
        .rsp_hdr_buf_dina (rsp_hdr_buf_dina),

        .i_clr_debug_sts               (clr_debug_sts),
        .pkt_dropped_sticky            (debug_sts_reg[0][0]),
        .req_pkt_buf_almost_full_sticky(debug_sts_reg[0][1]),
        .rsp_pkt_buf_almost_full_sticky(debug_sts_reg[0][2]),
        .pkt_desc_fifo_1_unoccupied_r  (debug_sts_reg[1][15:0]),
        .pkt_desc_fifo_1_full_sticky   (debug_sts_reg[0][3]),
        .req_pkt_desc_fifo_unoccupied_r(debug_sts_reg[2][15:0]),
        .req_pkt_desc_fifo_full_sticky (debug_sts_reg[0][4]),
        .rsp_pkt_desc_fifo_unoccupied_r(debug_sts_reg[2][31:16]),
        .rsp_pkt_desc_fifo_full_sticky (debug_sts_reg[0][5]),
        .pkt_desc_fifo_3_empty         (debug_sts_reg[0][16]),
        .req_pkt_desc_fifo_empty       (req_pkt_desc_fifo_empty),
        .rsp_pkt_desc_fifo_empty       (rsp_pkt_desc_fifo_empty),
        .pkt_fcs_err_sticky            (debug_sts_reg[0][29]),
        .pkt_crc_err_sticky            (debug_sts_reg[0][28])
    );

    always @(posedge core_clk) begin
        if (core_rst) req_pkt_desc_fifo_empty_r1 <= 1;
        else req_pkt_desc_fifo_empty_r1 <= req_pkt_desc_fifo_empty;
    end

    assign debug_sts_reg[0][17] = req_pkt_desc_fifo_empty_r1;
    assign debug_sts_reg[0][18] = rsp_pkt_desc_fifo_empty;
    // ----------------------------------------------------------------------------------------------

    // ======================== 请求包缓冲区 sdpram ========================
    xpm_memory_sdpram #(  // Common module parameters
        .MEMORY_SIZE      (REQ_BUF_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                  //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),          //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                //do not change
        .MEMORY_INIT_FILE ("none"),                  //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                      //string;
        .WAKEUP_TIME      ("disable_sleep"),         //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters                   
        .WRITE_DATA_WIDTH_A(512),                  //positive integer
        .BYTE_WRITE_WIDTH_A(8),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_REQ_BUF_ADDR_WIDTH), //positive integer

        // Port B module parameters                   
        .READ_DATA_WIDTH_B (512),                   //positive integer
        .ADDR_WIDTH_B      (C_REQ_BUF_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 
    ) xpm_req_pkt_buf (
        // Common module ports
        .sleep(1'b0),  //do not change

        // Port A module ports
        .clka          (core_clk),
        .ena           (req_pkt_buf_ena),
        .wea           (req_pkt_buf_wea),
        .addra         (req_pkt_buf_addra),
        .dina          (req_pkt_buf_dina),
        .injectsbiterra(1'b0),               //do not change
        .injectdbiterra(1'b0),               //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (req_pkt_buf_enb),
        .regceb  (1'b1),
        .addrb   (req_pkt_buf_addrb),
        .doutb   (req_pkt_buf_doutb),
        .sbiterrb(),                   //do not change
        .dbiterrb()                    //do not change
    );

    // ======================== 请求包头缓冲区 sdpram ========================
    xpm_memory_sdpram #(
        // Common module parameters
        .MEMORY_SIZE      (HDR_BUF_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                  //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),          //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                //do not change
        .MEMORY_INIT_FILE ("none"),                  //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                      //string;
        .WAKEUP_TIME      ("disable_sleep"),         //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(512),                  //positive integer
        .BYTE_WRITE_WIDTH_A(512),                  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_HDR_BUF_ADDR_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (512),                   //positive integer
        .ADDR_WIDTH_B      (C_HDR_BUF_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer，读数据输出延迟一个周期
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 

    ) xpm_req_hdr_buf (
        // Common module ports
        .sleep(1'b0),  //do not change

        // Port A module ports
        .clka          (core_clk),
        .ena           (req_hdr_buf_ena),
        .wea           (req_hdr_buf_wea[0]),
        .addra         (req_hdr_buf_addra),
        .dina          (req_hdr_buf_dina),
        .injectsbiterra(1'b0),                //do not change
        .injectdbiterra(1'b0),                //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (req_hdr_buf_enb),
        .regceb  (1'b1),
        .addrb   (req_hdr_buf_addrb),
        .doutb   (req_hdr_buf_doutb),
        .sbiterrb(),                   //do not change
        .dbiterrb()                    //do not change
    );

    // 若是包第一拍或者ipv6包，则qpid字段掩码全1，否则全0
    assign req_hdr_qpid_buf_wea[2:0] = (req_hdr_buf_ena == 1'b1 && (req_hdr_buf_addra[0] == 1'b0 || ipv6_valid == 1'b1)) ? 3'h7 : 3'h0;
    // 若是包第一拍，则opcode字段掩码为1，否则为0
    assign req_hdr_qpid_buf_wea[3] = (req_hdr_buf_ena == 1'b1 && req_hdr_buf_addra[0] == 1'b0) ? 1'b1 : 1'b0;
    // (req_hdr_buf_addra[0] == 1'b0)表示为包第一拍，(req_hdr_buf_addra[0] == 1'b1)表示为包第二拍
    // {req_hdr_buf_dina[47*8+:8], req_hdr_buf_dina[48*8+:8], req_hdr_buf_dina[49*8+:8]}是bth头部的QP字段
    assign req_hdr_qpid_buf_dina     = (req_hdr_buf_addra[0] == 1'b0) ? 
                                        {{req_hdr_buf_dina[47*8+:8], req_hdr_buf_dina[48*8+:8], req_hdr_buf_dina[49*8+:8]} >= 24'd1 ? {req_hdr_buf_dina[47*8+:8], req_hdr_buf_dina[48*8+:8], req_hdr_buf_dina[49*8+:8]} - 24'd1 : 24'd0} 
                                        : {{req_hdr_buf_dina[3*8+:8], req_hdr_buf_dina[4*8+:8], req_hdr_buf_dina[5*8+:8]} >= 24'd1 ? {req_hdr_buf_dina[3*8+:8], req_hdr_buf_dina[4*8+:8], req_hdr_buf_dina[5*8+:8]} - 24'd1 : 24'd0};


    // 由于包头存储为两拍对齐，req_hdr_buf_addra[0] == 1'b1表示新包头的第二拍，这时将rkey字段存入相应bram
    assign req_hdr_rkey_buf_ena = (req_hdr_buf_ena == 1'b1 && req_hdr_buf_addra[0] == 1'b1) ? 1'b1 : 1'b0;
    assign req_hdr_rkey_buf_wea = (req_hdr_buf_ena == 1'b1 && req_hdr_buf_addra[0] == 1'b1) ? 4'hF : 4'h0;
    assign req_hdr_rkey_buf_dina = (req_hdr_buf_ena == 1'b1 && ipv6_valid == 1'b0) ? {ipv4_rkey_lsb, req_hdr_buf_dina[7:0], req_hdr_buf_dina[15:8]} : {req_hdr_buf_dina[18*8+:8], req_hdr_buf_dina[19*8+:8], req_hdr_buf_dina[20*8+:8], req_hdr_buf_dina[21*8+:8]};

    assign ether_type_ipv4 = {req_hdr_buf_dina[12*8+:8], req_hdr_buf_dina[13*8+:8]};
    assign bth_opcode = (ether_type_ipv4 == 16'h0800) ? req_hdr_buf_dina[42*8+:8] : req_hdr_buf_dina[62*8+:8];

    always @(posedge core_clk) begin
        if (core_rst) begin
            ipv6_valid    <= 1'b0;
            ipv4_rkey_lsb <= 'b0;
        end else begin

            if (req_hdr_buf_ena == 1'b1 && req_hdr_buf_addra[0] == 1'b0 && ether_type_ipv4 == 16'h0800) ipv4_rkey_lsb <= {req_hdr_buf_dina[62*8+:8], req_hdr_buf_dina[63*8+:8]};

            if (req_hdr_buf_ena == 1'b1 && req_hdr_buf_addra[0] == 1'b0 && ether_type_ipv4 != 16'h0800) ipv6_valid <= 1'b1;
            else if (req_hdr_buf_ena == 1'b1 && req_hdr_buf_addra[0] == 1'b1) ipv6_valid <= 1'b0;

        end
    end

    // ======================== 请求包头 {BTH头部 bth_opcode 字段, BTH头部 Dest QP 字段} sdpram ========================
    xpm_memory_sdpram #(
        // Common module parameters
        .MEMORY_SIZE      (HDR_BUF_QPID_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                       //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),               //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                     //do not change
        .MEMORY_INIT_FILE ("none"),                       //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                           //string;
        .WAKEUP_TIME      ("disable_sleep"),              //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),                   //positive integer
        .BYTE_WRITE_WIDTH_A(8),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_CFG_MEM_ADDR_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (32),                    //positive integer
        .ADDR_WIDTH_B      (C_CFG_MEM_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 

    ) xpm_req_hdr_buf_qpid (
        // Common module ports
        .sleep(1'b0),  //do not change

        // Port A module ports
        .clka(core_clk),
        .ena (req_hdr_buf_ena),
        .wea (req_hdr_qpid_buf_wea),

        // 与包头索引对齐，若第2个包头（两拍）（包头索引为1）存在0x02和0x03中，即第3个、第4个单元，
        // 则{bth_opcode, req_hdr_qpid_buf_dina}存在第1个单元（与包头索引对齐），即0x01中
        .addra         (req_hdr_buf_addra[C_HDR_BUF_ADDR_WIDTH-1:1]),
        .dina          ({bth_opcode, req_hdr_qpid_buf_dina}),          // 高8位存opcode，低24位存qpid
        .injectsbiterra(1'b0),                                         //do not change
        .injectdbiterra(1'b0),                                         //do not change


        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (req_hdr_buf_enb),
        .regceb  (1'b1),
        .addrb   (req_hdr_buf_addrb[C_HDR_BUF_ADDR_WIDTH-1:1]),
        .doutb   (req_hdr_qpid_buf_doutb),                       // {bth_opcode, qpid}
        .sbiterrb(),                                             //do not change
        .dbiterrb()                                              //do not change
    );

    // ======================== 请求包头 reth头部 rkey 字段 sdpram ========================
    xpm_memory_sdpram #(
        // Common module parameters
        .MEMORY_SIZE      (HDR_BUF_RKEY_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                       //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),               //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                     //do not change
        .MEMORY_INIT_FILE ("none"),                       //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                           //string;
        .WAKEUP_TIME      ("disable_sleep"),              //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),                   //positive integer
        .BYTE_WRITE_WIDTH_A(8),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_CFG_MEM_ADDR_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (32),                    //positive integer
        .ADDR_WIDTH_B      (C_CFG_MEM_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 

    ) xpm_req_hdr_buf_rkey (
        // Common module ports
        .sleep(1'b0),  //do not change

        // Port A module ports
        .clka          (core_clk),
        .ena           (req_hdr_rkey_buf_ena),
        .wea           (req_hdr_rkey_buf_wea),
        .addra         (req_hdr_buf_addra[C_HDR_BUF_ADDR_WIDTH-1:1]),  // 与包头索引对齐，第一个包为0,第2个包为1...
        .dina          (req_hdr_rkey_buf_dina),                        // reth头部 rkey
        .injectsbiterra(1'b0),                                         //do not change
        .injectdbiterra(1'b0),                                         //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (req_hdr_buf_enb),
        .regceb  (1'b1),
        .addrb   (req_hdr_buf_addrb[C_HDR_BUF_ADDR_WIDTH-1:1]),
        .doutb   (req_hdr_rkey_buf_doutb),
        .sbiterrb(),                                             //do not change
        .dbiterrb()                                              //do not change
    );

    // ======================== 响应包缓冲区 sdpram ========================
    xpm_memory_sdpram #(  // Common module parameters
        .MEMORY_SIZE      (RSP_BUF_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                  //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),          //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                //do not change
        .MEMORY_INIT_FILE ("none"),                  //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                      //string;
        .WAKEUP_TIME      ("disable_sleep"),         //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters                   
        .WRITE_DATA_WIDTH_A(512),                  //positive integer
        .BYTE_WRITE_WIDTH_A(8),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_RSP_BUF_ADDR_WIDTH), //positive integer

        // Port B module parameters                   
        .READ_DATA_WIDTH_B (512),                   //positive integer
        .ADDR_WIDTH_B      (C_RSP_BUF_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 
    ) xpm_rsp_pkt_buf (
        // Common module ports
        .sleep(1'b0),  //do not change

        // Port A module ports
        .clka          (core_clk),
        .ena           (rsp_pkt_buf_ena),
        .wea           (rsp_pkt_buf_wea),
        .addra         (rsp_pkt_buf_addra),
        .dina          (rsp_pkt_buf_dina),
        .injectsbiterra(1'b0),               //do not change
        .injectdbiterra(1'b0),               //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (rsp_pkt_buf_enb),    // 读使能
        .regceb  (1'b1),
        .addrb   (rsp_pkt_buf_addrb),  // 读地址
        .doutb   (rsp_pkt_buf_doutb),
        .sbiterrb(),                   //do not change
        .dbiterrb()                    //do not change
    );


    assign rsp_hdr_qpid_buf_wea = (rsp_hdr_buf_ena == 1'b1 && (rsp_hdr_buf_addra[0] == 1'b0 || rsp_ipv6_valid == 1'b1)) ? 4'hF : 4'h0;
    assign rsp_hdr_qpid_buf_dina = (rsp_hdr_buf_addra[0] == 1'b0) ? {{rsp_hdr_buf_dina[47*8+:8], rsp_hdr_buf_dina[48*8+:8], rsp_hdr_buf_dina[49*8+:8]} >= 24'd1 ? {rsp_hdr_buf_dina[47*8+:8], rsp_hdr_buf_dina[48*8+:8], rsp_hdr_buf_dina[49*8+:8]} - 24'd1 : 24'd0} 
                                    : {{rsp_hdr_buf_dina[3*8+:8], rsp_hdr_buf_dina[4*8+:8], rsp_hdr_buf_dina[5*8+:8]} >= 24'd1 ? {rsp_hdr_buf_dina[3*8+:8], rsp_hdr_buf_dina[4*8+:8], rsp_hdr_buf_dina[5*8+:8]} - 24'd1 : 24'd0};
    assign rsp_ether_type_ipv4 = {rsp_hdr_buf_dina[12*8+:8], rsp_hdr_buf_dina[13*8+:8]};

    always @(posedge core_clk) begin
        if (core_rst) begin
            rsp_ipv6_valid <= 1'b0;
        end else begin

            if (rsp_hdr_buf_ena == 1'b1 && rsp_hdr_buf_addra[0] == 1'b0 && rsp_ether_type_ipv4 != 16'h0800) rsp_ipv6_valid <= 1'b1;
            else if (rsp_hdr_buf_ena == 1'b1 && rsp_hdr_buf_addra[0] == 1'b1) rsp_ipv6_valid <= 1'b0;

        end
    end

    // ======================== 响应包头缓冲区 sdpram ========================
    xpm_memory_sdpram #(
        // Common module parameters
        .MEMORY_SIZE      (HDR_BUF_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                  //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),          //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                //do not change
        .MEMORY_INIT_FILE ("none"),                  //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                      //string;
        .WAKEUP_TIME      ("disable_sleep"),         //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(512),                  //positive integer
        .BYTE_WRITE_WIDTH_A(512),                  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_HDR_BUF_ADDR_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (512),                   //positive integer
        .ADDR_WIDTH_B      (C_HDR_BUF_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 

    ) xpm_rsp_hdr_buf (
        // Common module ports
        .sleep(1'b0),  //do not change

        // Port A module ports
        .clka          (core_clk),
        .ena           (rsp_hdr_buf_ena),
        .wea           (rsp_hdr_buf_wea[0]),  // 因为 BYTE_WRITE_WIDTH_A = 512，WRITE_DATA_WIDTH_A = 512，所以wea宽度=1
        .addra         (rsp_hdr_buf_addra),   // 两拍对齐，包头存两拍
        .dina          (rsp_hdr_buf_dina),
        .injectsbiterra(1'b0),                //do not change
        .injectdbiterra(1'b0),                //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (rsp_hdr_buf_enb),
        .regceb  (1'b1),
        .addrb   (rsp_hdr_buf_addrb),
        .doutb   (rsp_hdr_buf_doutb),
        .sbiterrb(),                   //do not change
        .dbiterrb()                    //do not change
    );

    // ======================== 响应包头 {8'h0, qpid} sdpram ========================
    xpm_memory_sdpram #(
        // Common module parameters
        .MEMORY_SIZE      (HDR_BUF_QPID_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                       //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),               //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                     //do not change
        .MEMORY_INIT_FILE ("none"),                       //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                           //string;
        .WAKEUP_TIME      ("disable_sleep"),              //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),                   //positive integer
        .BYTE_WRITE_WIDTH_A(8),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_CFG_MEM_ADDR_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (32),                    //positive integer
        .ADDR_WIDTH_B      (C_CFG_MEM_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 

    ) xpm_rsp_hdr_buf_qpid (
        // Common module ports
        .sleep(1'b0),  //do not change

        // Port A module ports
        .clka          (core_clk),
        .ena           (rsp_hdr_buf_ena),
        .wea           (rsp_hdr_qpid_buf_wea),
        .addra         (rsp_hdr_buf_addra[C_HDR_BUF_ADDR_WIDTH-1:1]),
        .dina          ({8'h0, rsp_hdr_qpid_buf_dina}),                // 高8位存0，低24位存qpid
        .injectsbiterra(1'b0),                                         //do not change
        .injectdbiterra(1'b0),                                         //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (rsp_hdr_buf_enb),
        .regceb  (1'b1),
        .addrb   (rsp_hdr_buf_addrb[C_HDR_BUF_ADDR_WIDTH-1:1]),
        .doutb   (rsp_hdr_qpid_buf_doutb),
        .sbiterrb(),                                             //do not change
        .dbiterrb()                                              //do not change
    );


    rx_req_hdr_val_sv #(
        .C_SIM_DEBUG(C_SIM_DEBUG),
        .C_EN_INITIATOR_LITE(C_EN_INITIATOR_LITE),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .C_NUM_QP(C_NUM_QP),
        .C_PD_INDX_WIDTH(C_PD_INDX_WIDTH),
        .C_NUM_PD(C_NUM_PD),
        .PKT_DESC_FIFO_DATA_WIDTH(PKT_DESC_FIFO_DATA_WIDTH),
        .C_HDR_BUF_ADDR_WIDTH(C_HDR_BUF_ADDR_WIDTH),
        .C_REQ_BUF_ADDR_WIDTH(C_REQ_BUF_ADDR_WIDTH),
        .C_RSP_BUF_ADDR_WIDTH(C_RSP_BUF_ADDR_WIDTH),
        .C_EN_IETH_IMMDT(C_EN_IETH_IMMDT)
    ) u_rx_req_hdr_val (
        .clk                         (core_clk),
        .reset                       (core_rst),
        .req_pkt_desc_fifo_dout      (req_pkt_desc_fifo_dout),
        .req_pkt_desc_fifo_empty     (req_pkt_desc_fifo_empty),
        .rd_req_pkt_desc_fifo        (rd_req_pkt_desc_fifo),
        .req_hdr_buf_addrb           (req_hdr_buf_addrb),
        .req_hdr_buf_enb             (req_hdr_buf_enb),
        .req_hdr_buf_doutb           (req_hdr_buf_doutb),
        .req_hdr_qpid_buf_doutb      (req_hdr_qpid_buf_doutb),
        .req_hdr_rkey_buf_doutb      (req_hdr_rkey_buf_doutb),
        .i_bypass_en                 (i_depkt_bypass_en),
        .i_err_buf_en                (i_req_err_buf_en),
        .i_mac_node_addr             (i_mac_node_addr),
        .i_ipv4_node_addr            (i_ipv4_node_addr),
        .i_ipv6_node_addr            (i_ipv6_node_addr),
        .i_num_qp_enabled            (i_num_qp_enabled),
        .i_rnr_nak_tval              (i_rnr_nak_tval),
        .i_flow_credits              (i_flow_credits),
        .o_inc_dup_pkt_cnt           (o_inc_dup_pkt_cnt),
        .o_out_nack_pkt_cnt          (o_out_nack_pkt_cnt),
        .o_last_in_req_pkt_opcode    (o_last_in_req_pkt_opcode),
        .o_last_in_req_pkt_qpid      (o_last_in_req_pkt_qpid),
        .o_last_in_req_pkt_psn       (o_last_in_req_pkt_psn),
        //.o_rq_full               (o_rq_full),
        .o_rq_full_idx               (o_rq_full_idx),
        .o_rq_full_wr_en             (o_rq_full_wr_en),
        .o_rq_full_val               (o_rq_full_val),
        .o_rq_full_reset_idx         (o_rq_full_reset_idx),
        .o_rq_full_reset_wr_en       (o_rq_full_reset_wr_en),
        .o_rq_full_reset_val         (o_rq_full_reset_val),
        .o_req_indicator             (o_req_indicator),
        .o_physical_addr             (o_physical_addr),
        .o_dma_len                   (o_dma_len),
        .o_rd_req_pmtu               (o_rd_req_pmtu),
        .o_rsp_last_psn              (o_rsp_last_psn),
        //.i_rd_req_fifo_full      (i_rd_req_fifo_full),
        .i_rd_req_fifo_full_set_idx  (i_rd_req_fifo_full_set_idx),
        .i_rd_req_fifo_full_set      (i_rd_req_fifo_full_set),
        .i_rd_req_fifo_full_reset_idx(i_rd_req_fifo_full_reset_idx),
        .i_rd_req_fifo_full_reset    (i_rd_req_fifo_full_reset),
        .i_pd_pd_num                 (i_pd_pd_num),
        .i_pd_virt_addr_0            (i_pd_virt_addr_0),
        .i_pd_virt_addr_1            (i_pd_virt_addr_1),
        .i_pd_buf_base_addr_0        (i_pd_buf_base_addr_0),
        .i_pd_buf_base_addr_1        (i_pd_buf_base_addr_1),
        .i_pd_buf_r_key              (i_pd_buf_r_key),
        .i_pd_w_r_buf_len            (i_pd_w_r_buf_len),
        .i_pd_access_type            (i_pd_access_type),
        .o_qp_conf_idx               (o_qp_req_conf_idx),
        .o_pd_conf_idx               (o_pd_req_conf_idx),
        .i_qp_conf                   (i_qp_req_conf),
        .qp_conf_vld                 (qp_req_conf_vld),
        .qp_conf_en                  (qp_req_conf_en),
        .pd_conf_en                  (pd_req_conf_en),
        .i_rq_buf_ba                 (i_rq_buf_ba),
        .rq_buf_ba_en                (rq_buf_ba_en),
        .i_rq_buf_ca                 (i_rq_buf_ca),
        .o_rq_buf_ca                 (o_rq_buf_ca),
        .rq_buf_ca_en                (rq_buf_ca_en),
        .rq_buf_ca_we                (rq_buf_ca_we),
        .i_rq_wrptr_db_addr          (i_rq_wrptr_db_addr),
        .rq_wrptr_db_addr_en         (rq_wrptr_db_addr_en),
        .i_q_depth                   (i_q_depth),
        .q_depth_en                  (q_depth_en),
        .i_rq_ci_db                  (i_rq_ci_db),
        .rq_ci_db_en                 (rq_ci_db_en),
        .i_last_rq_req               (i_last_rq_req),
        .o_last_rq_req               (o_last_rq_req),
        .last_rq_req_en              (last_rq_req_en),
        .last_rq_req_we              (last_rq_req_we),
        .i_qp_pd_num                 (i_qp_pd_num),
        .qp_pd_num_en                (qp_pd_num_en),
        .i_stat_qp_msn               (i_stat_qp_msn),
        .o_stat_qp_msn               (o_stat_qp_msn),
        .stat_qp_msn_en              (stat_qp_msn_en),
        .stat_qp_msn_we              (stat_qp_msn_we),
        .i_remote_qp_mac_addr        (i_qp_req_remote_qp_mac_addr),
        .remote_qp_mac_addr_en       (qp_req_remote_qp_mac_addr_en),
        .i_remote_qp_ip_addr         (i_qp_req_remote_qp_ip_addr),
        .remote_qp_ip_addr_en        (qp_req_remote_qp_ip_addr_en),
        .i_rq_wrptr_db               (i_rq_wrptr_db),
        .o_rq_wrptr_db               (o_rq_wrptr_db),
        .rq_wrptr_db_en              (rq_wrptr_db_en),
        .rq_wrptr_db_we              (rq_wrptr_db_we),
        //.i_qp_fatal              (qp_fatal),
        .i_qp_fatal_wr_en            (o_qp_fatal_wr_en),
        .i_qp_fatal_wr_idx           (o_qp_fatal_wr_idx),
        .i_qp_fatal_wr_data          (o_qp_fatal_wr_data),
        .o_qp_fatal_idx              (qp_req_fatal_idx),
        .set_qp_fatal                (set_qp_req_fatal),
        .rsp_qp_fatal_idx            (qp_rsp_fatal_idx),
        .set_rsp_qp_fatal            (set_qp_rsp_fatal),
        .set_rnr_nack_gen_intr       (set_rnr_nack_gen_intr),
        .i_qp_fatal_wqe_proc         (i_qp_fatal_wqe_proc),
        .i_qp_fatal_wqe_proc_vld     (i_qp_fatal_wqe_proc_vld),
        .o_qp_fatal_wqe_proc_rdy     (o_qp_fatal_wqe_proc_rdy),
        .i_qp_fatal_rsp_hndl         (i_qp_fatal_rsp_hndl),
        .i_qp_fatal_rsp_hndl_vld     (i_qp_fatal_rsp_hndl_vld),
        .o_qp_fatal_rsp_hndl_rdy     (o_qp_fatal_rsp_hndl_rdy),
        .o_ob_rsp_qpid               (o_ob_rsp_qpid),
        .o_ob_rsp_psn                (o_ob_rsp_psn),
        .o_ob_rsp_aeth_syndrome      (o_ob_rsp_aeth_syndrome),
        .o_ob_rsp_aeth_msn           (o_ob_rsp_aeth_msn),
        .o_ob_rsp_expl_ack           (o_ob_rsp_expl_ack),
        .o_ob_rsp_vld                (o_ob_rsp_vld),
        .ob_rsp_fifo_full            (ob_rsp_fifo_full),
        .pkt_discarded               (pkt_discarded),
        .req_pkt_id                  (req_pkt_id),
        .req_qpid                    (req_qpid),
        .req_pkt_type                (req_pkt_type),
        .req_pkt_ip_ver              (req_pkt_ip_ver),
        .req_rq_pi_db                (req_rq_pi_db),
        .req_dest_addr               (req_dest_addr),
        .req_buf_addr_offset         (req_buf_addr_offset),
        .req_pkt_len                 (req_pkt_len),
        .req_imm_data                (req_imm_data),
        .req_tnfr_vld                (req_tnfr_vld),
        .req_ieth_present            (req_ieth_present),
        .req_immdt_present           (req_immdt_present),
        .ieth_immdt_fifo_data        (ieth_immdt_fifo_data),
        .ieth_immdt_fifo_wr_en       (ieth_immdt_fifo_wr_en),
        .rdy_to_req_tnfr             (rdy_to_req_tnfr),
        //.ecn_vect                (rx_req_ecn_vect),
        .ecn_vect_idx                (rx_req_ecn_vect_idx),
        .ecn_vect_vld                (rx_req_ecn_update_flg),
        .cnp_pkt_rcvd                (cnp_pkt_rcvd),
        .cnp_pkt_qpid                (cnp_pkt_qpid),
        .o_all_rd_req_cnt            (o_all_rd_req_cnt),
        .o_good_rd_req_cnt           (o_good_rd_req_cnt),
        .o_dup_rd_req_cnt            (o_dup_rd_req_cnt),
        //.i_qp_enable             (i_qp_enable), 
        .i_qp_enable_wr_en           (i_qp_enable_wr_en),
        .i_qp_enable_wr_idx          (i_qp_enable_wr_idx),
        .i_qp_enable_wr_data         (i_qp_enable_wr_data),

        .waiting_for_rq_update          (waiting_for_rq_update),
        .waiting_for_rq_update_on_qp_num(waiting_for_rq_update_on_qp_num),
        .rnr_window_flag_reg            (rnr_window_flag_reg),

        .i_clr_debug_sts          (clr_debug_sts),
        .rq_buf_full_sticky       (debug_sts_reg[0][12]),
        .qp_fatal_req_hv_sticky   (debug_sts_reg[0][13]),
        .rq_buf_full_event_cnt    (debug_sts_reg[10]),
        .rnr_window_cnt           (debug_sts_reg[11]),
        .init_start               (init_start),
        .init_count               (init_count),
        .qp_under_recovery_wr_en  (qp_under_recovery_wr_en),
        .qp_under_recovery_wr_idx (qp_under_recovery_wr_idx),
        .qp_under_recovery_wr_data(qp_under_recovery_wr_data)
    );

    reg_pipe #(
        .C_DATA_WIDTH(C_QP_INDX_WIDTH + C_M_AXI_ADDR_WIDTH + C_REQ_BUF_ADDR_WIDTH + 16 + 89 + 1 + 2 + 4)
    )  //16 for req_rq_pi_db
        u_reg_pipe_1 (
        .clk    (core_clk),
        .reset  (core_rst),
        .din    ({req_ieth_present, req_immdt_present, req_qpid, req_pkt_id, req_pkt_type, req_pkt_ip_ver, req_rq_pi_db, req_dest_addr, req_buf_addr_offset, req_pkt_len, req_imm_data}),
        .vld_in (req_tnfr_vld),
        .rdy_out(rdy_to_req_tnfr),
        .dout   ({o_req_ieth_present, o_req_immdt_present, o_req_qpid, o_req_pkt_id, o_req_pkt_type, o_req_pkt_ip_ver, o_req_rq_pi_db, o_req_dest_addr, o_req_buf_addr_offset, o_req_pkt_len, o_req_imm_data}),
        .vld_out(o_req_tnfr_vld),
        .rdy_in (i_rdy_to_req_tnfr)
    );

    reg_pipe #(
        .C_DATA_WIDTH(64)
    ) u_reg_pipe_err_data (
        .clk    (core_clk),
        .reset  (core_rst),
        .din    ({rsp_pkt_id, rsp_imm_data}),
        .vld_in (req_err_tnfr_vld),
        .rdy_out(rdy_to_req_err_tnfr),
        .dout   ({o_req_err_pkt_id, o_req_err_imm_data}),
        .vld_out(o_req_err_tnfr_vld),
        .rdy_in (i_rdy_to_req_err_tnfr)
    );

    rx_req_axi_mstr #(
        .C_SIM_DEBUG(C_SIM_DEBUG),
        .C_EN_INITIATOR_LITE(C_EN_INITIATOR_LITE),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_AWUSER_WIDTH(C_M_AXI_AWUSER_WIDTH),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .C_NUM_QP(C_NUM_QP),
        .C_REQ_BUF_ADDR_WIDTH(C_REQ_BUF_ADDR_WIDTH),
        .C_EN_IETH_IMMDT(C_EN_IETH_IMMDT)
    ) u_rx_req_axi_mstr (
        .clk                     (core_clk),
        .reset                   (core_rst),
        .req_pkt_id              (o_req_pkt_id),
        .req_pkt_type            (o_req_pkt_type),
        .req_pkt_ip_ver          (o_req_pkt_ip_ver),
        .req_rq_pi_db            (o_req_rq_pi_db),
        .req_dest_addr           (o_req_dest_addr),
        .req_qpid                (o_req_qpid),
        .req_buf_addr_offset     (o_req_buf_addr_offset),
        .req_pkt_len             (o_req_pkt_len),
        .req_imm_data            (o_req_imm_data),
        .req_tnfr_vld            (o_req_tnfr_vld),
        .req_ieth_present        (o_req_ieth_present),
        .req_immdt_present       (o_req_immdt_present),
        .ieth_immdt_fifo_wr_data (ieth_immdt_fifo_data),
        .ieth_immdt_fifo_wr_en   (ieth_immdt_fifo_wr_en),
        .rdy_to_req_tnfr         (i_rdy_to_req_tnfr),
        .rsp_pkt_id              (o_req_err_pkt_id),
        .rsp_imm_data            (o_req_err_imm_data),
        .rsp_tnfr_vld            (o_req_err_tnfr_vld),
        .rdy_to_rsp_tnfr         (i_rdy_to_req_err_tnfr),
        .i_bypass_buf_ba         (i_bypass_buf_ba),
        .i_bypass_buf_sz         (i_bypass_buffer_sz),
        .i_bypass_num_bufs       (i_bypass_num_bufs),
        .o_bypass_buf_ca         (),
        .o_bypass_buf_wrptr      (o_bypass_buf_wrptr),
        .i_err_buf_ba            (i_req_err_pkt_buf_ba),
        .i_err_buf_sz            (i_req_err_buffer_sz),
        .i_err_num_bufs          (i_req_err_num_bufs),
        .o_err_buf_ca            (),
        .o_err_buf_wrptr         (o_req_err_buf_wrptr),
        .i_in_err_sts_q_ba       (i_in_errsts_q_ba),
        .i_in_err_sts_q_sz       (16'd8),
        .i_in_err_sts_q_depth    (i_in_errsts_q_sz),
        .o_in_err_sts_q_ca       (),
        .o_in_err_sts_q_wrptr    (o_in_errsts_q_wrptr),
        .o_bypass_pkt_cnt        (),
        .incr_inv_pkt_cnt        (incr_inv_req_pkt_cnt),
        .o_mad_pkt_cnt           (o_inc_mad_cnt),
        .o_send_req_cnt          (),
        .o_send_pkt_cnt          (o_inc_send_cnt),
        .xrnic_en_pulse          (xrnic_en_pulse),
        .set_bypass_pkt_rcvd_intr(set_bypass_pkt_rcvd_intr),
        .set_pkt_valdn_err_intr  (set_req_pkt_valdn_err_intr),
        .set_mad_pkt_rcvd_intr   (set_mad_pkt_rcvd_intr),
        .set_qp_req_rcvd_intr_1  (set_qp_pkt_rcvd_intr_1),
        .intr_qp_indx_1          (pkt_rcvd_qpid_1),
        .set_qp_req_rcvd_intr_2  (set_qp_pkt_rcvd_intr_2),
        .intr_qp_indx_2          (pkt_rcvd_qpid_2),
        .set_qp_fatal_intr       (set_fatal_err_intr),
        .req_pkt_buf_addra       (req_pkt_buf_addra),
        .req_pkt_buf_enb         (req_pkt_buf_enb),
        .req_pkt_buf_addrb       (req_pkt_buf_addrb),
        .req_pkt_buf_doutb       (req_pkt_buf_doutb),
        .rq_db_data_valid        (o_rq_db_data_valid),
        .rq_db_addr              (o_rq_db_addr),
        .rq_db_data              (o_rq_db_data[15:0]),
        .rq_db_data_ack          (i_rq_db_data_ack),
        .m_axi_awid              (rx_req_axi_awid),
        .m_axi_awaddr            (rx_req_axi_awaddr),
        .m_axi_awuser            (rx_req_axi_awuser),
        .m_axi_awlen             (rx_req_axi_awlen),
        .m_axi_awsize            (rx_req_axi_awsize),
        .m_axi_awburst           (rx_req_axi_awburst),
        .m_axi_awcache           (rx_req_axi_awcache),
        .m_axi_awprot            (rx_req_axi_awprot),
        .m_axi_awvalid           (rx_req_axi_awvalid),
        .m_axi_awready           (rx_req_axi_awready),
        .m_axi_wdata             (rx_req_axi_wdata),
        .m_axi_wstrb             (rx_req_axi_wstrb),
        .m_axi_wlast             (rx_req_axi_wlast),
        .m_axi_wvalid            (rx_req_axi_wvalid),
        .m_axi_wready            (rx_req_axi_wready),
        .m_axi_awlock            (rx_req_axi_awlock),
        .m_axi_bid               (rx_req_axi_bid),
        .m_axi_bresp             (rx_req_axi_bresp),
        .m_axi_bvalid            (rx_req_axi_bvalid),
        .m_axi_bready            (rx_req_axi_bready),
        .m_axi_arid              (rx_req_axi_arid),
        .m_axi_araddr            (rx_req_axi_araddr),
        .m_axi_arlen             (rx_req_axi_arlen),
        .m_axi_arsize            (rx_req_axi_arsize),
        .m_axi_arburst           (rx_req_axi_arburst),
        .m_axi_arcache           (rx_req_axi_arcache),
        .m_axi_arprot            (rx_req_axi_arprot),
        .m_axi_arvalid           (rx_req_axi_arvalid),
        .m_axi_arready           (rx_req_axi_arready),
        .m_axi_rid               (rx_req_axi_rid),
        .m_axi_rdata             (rx_req_axi_rdata),
        .m_axi_rresp             (rx_req_axi_rresp),
        .m_axi_rlast             (rx_req_axi_rlast),
        .m_axi_rvalid            (rx_req_axi_rvalid),
        .m_axi_rready            (rx_req_axi_rready),
        .m_axi_arlock            (rx_req_axi_arlock),

        .i_rq_pi_db_hw_hndshk         (i_rq_pi_db_hw_hndshk),
        .i_connect_io_qp_rq_pi_db_wptr(i_connect_io_qp_rq_pi_db_wptr),
        .i_connect_io_qpid            (i_connect_io_qpid),
        .i_hw_hndshk_disable_to_0     (i_hw_hndshk_disable_to_0),
        .o_connect_io_qp_rq_pi_db_rdy (o_connect_io_qp_rq_pi_db_rdy),

        .rq_wrptr_db_we(rq_wrptr_db_we),

        .free_run_cnt(free_run_cnt),

        .i_clr_debug_sts                (clr_debug_sts),
        .axi_awchnl_fifo_unoccupied_r   (debug_sts_reg[3][15:0]),
        .axi_awchnl_max_wait_st         (debug_sts_reg[4][15:0]),
        .axi_wchnl_fifo_unoccupied_r    (debug_sts_reg[3][31:16]),
        .axi_wchnl_max_wait_st          (debug_sts_reg[4][31:16]),
        .post_axi_tnfr_fifo_unoccupied_r(debug_sts_reg[6][15:0]),
        .db_fifo_unoccupied_r           (debug_sts_reg[6][31:16]),
        .axi_bresp_max_wait_st          (debug_sts_reg[5][15:0]),
        .axi_awchnl_fifo_empty          (debug_sts_reg[0][19]),
        .axi_awchnl_fifo_full_sticky    (debug_sts_reg[0][6]),
        .axi_wchnl_fifo_empty           (debug_sts_reg[0][20]),
        .axi_wchnl_fifo_full_sticky     (debug_sts_reg[0][7]),
        .post_axi_tnfr_fifo_empty       (debug_sts_reg[0][23]),
        .post_axi_tnfr_fifo_full_sticky (debug_sts_reg[0][10]),
        .db_fifo_empty                  (debug_sts_reg[0][24]),
        .db_fifo_full_sticky            (debug_sts_reg[0][11]),

        .ieth_immdt_axis_tvalid(ieth_immdt_axis_tvalid),
        .ieth_immdt_axis_tlast (ieth_immdt_axis_tlast),
        .ieth_immdt_axis_tdata (ieth_immdt_axis_tdata),
        .ieth_immdt_axis_trdy  (ieth_immdt_axis_trdy)
    );

    rx_rsp_hdr_val #(
        .C_SIM_DEBUG(C_SIM_DEBUG),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .C_NUM_QP(C_NUM_QP),
        .PKT_DESC_FIFO_DATA_WIDTH(PKT_DESC_FIFO_DATA_WIDTH),
        .C_HDR_BUF_ADDR_WIDTH(C_HDR_BUF_ADDR_WIDTH),
        .C_RSP_BUF_ADDR_WIDTH(C_RSP_BUF_ADDR_WIDTH),
        .C_OSQ_PSN_WIDTH(C_OSQ_PSN_WIDTH)
    ) u_rx_rsp_hdr_val (
        .clk                        (core_clk),
        .reset                      (core_rst),
        .rsp_pkt_desc_fifo_dout     (rsp_pkt_desc_fifo_dout),
        .rsp_pkt_desc_fifo_empty    (rsp_pkt_desc_fifo_empty),
        .rd_rsp_pkt_desc_fifo       (rd_rsp_pkt_desc_fifo),
        .rsp_hdr_buf_addrb          (rsp_hdr_buf_addrb),
        .rsp_hdr_buf_enb            (rsp_hdr_buf_enb),
        .rsp_hdr_buf_doutb          (rsp_hdr_buf_doutb),
        .rsp_hdr_qpid_buf_doutb     (rsp_hdr_qpid_buf_doutb),
        .i_err_buf_en               (i_rsp_err_buf_en),
        .i_mac_node_addr            (i_mac_node_addr),
        .i_ipv4_node_addr           (i_ipv4_node_addr),
        .i_ipv6_node_addr           (i_ipv6_node_addr),
        .i_num_qp_enabled           (i_num_qp_enabled),
        .i_rnr_nak_rst_val          (i_rnr_nak_rst_val),
        .o_acknack_pkt_cnt          (o_inc_acknak_cnt),
        .o_inc_nack_pkt_cnt         (o_inc_nack_pkt_cnt),
        .o_last_in_rsp_pkt_opcode   (o_last_in_rsp_pkt_opcode),
        .o_last_in_rsp_pkt_qpid     (o_last_in_rsp_pkt_qpid),
        .o_last_in_rsp_pkt_psn      (o_last_in_rsp_pkt_psn),
        .o_qp_conf_idx              (o_qp_rsp_conf_idx),
        .i_qp_conf                  (i_qp_rsp_conf),
        .qp_conf_vld                (qp_rsp_conf_vld),
        .qp_conf_en                 (qp_rsp_conf_en),
        .i_stat_nak                 (i_stat_nak),
        .stat_nak_en                (stat_nak_en),
        .stat_nak_we                (stat_nak_we),
        .o_stat_nak                 (o_stat_nak),
        .i_stat_resp_psn            (i_stat_resp_psn),
        .stat_resp_psn_en           (stat_resp_psn_en),
        .stat_resp_psn_we           (stat_resp_psn_we),
        .o_stat_resp_psn            (o_stat_resp_psn),
        .i_sq_psn                   (i_sq_psn),
        .sq_psn_en                  (sq_psn_en),
        .i_remote_qp_mac_addr       (i_qp_rsp_remote_qp_mac_addr),
        .remote_qp_mac_addr_en      (qp_rsp_remote_qp_mac_addr_en),
        .i_remote_qp_ip_addr        (i_qp_rsp_remote_qp_ip_addr),
        .remote_qp_ip_addr_en       (qp_rsp_remote_qp_ip_addr_en),
        .i_qp_timeout               (i_qp_timeout),
        //.i_qp_fatal              (qp_fatal),
        .i_qp_fatal_wr_en           (o_qp_fatal_wr_en),
        .i_qp_fatal_wr_idx          (o_qp_fatal_wr_idx),
        .i_qp_fatal_wr_data         (o_qp_fatal_wr_data),
        .o_qp_fatal_idx             (qp_rsp_fatal_idx),
        .set_qp_fatal               (set_qp_rsp_fatal),
        .req_qp_fatal_idx           (qp_req_fatal_idx),
        .set_req_qp_fatal           (set_qp_req_fatal),
        .o_qp_mpsnbuf_idx           (o_qp_index_mpsnbuf),
        .mpsnbuf_en                 (o_max_epsn_req),
        .mpsnbuf_vld                (i_max_epsn_vld),
        //.i_mpsnbuf_empty         (i_mpsnbuf_empty),
        .i_mpsnbuf_empty_set_valid  (i_mpsnbuf_empty_set_valid),
        .i_mpsnbuf_empty_set_id     (i_mpsnbuf_empty_set_id),
        .i_mpsnbuf_empty_reset_valid(i_mpsnbuf_empty_reset_valid),
        .i_mpsnbuf_empty_reset_id   (i_mpsnbuf_empty_reset_id),
        //.i_qp_retried            (i_qp_retried),
        .i_qp_retried_set_valid     (i_qp_retried_set_valid),
        .i_qp_retried_set_id        (i_qp_retried_set_id),
        .i_qp_retried_reset_valid   (i_qp_retried_reset_valid),
        .i_qp_retried_reset_id      (i_qp_retried_reset_id),
        //.i_osq_empty             (i_osq_empty),
        .i_osq_empty_set_valid      (i_osq_empty_set_valid),
        .i_osq_empty_set_id         (i_osq_empty_set_id),
        .i_osq_empty_reset_valid    (i_osq_empty_reset_valid),
        .i_osq_empty_reset_id       (i_osq_empty_reset_id),
        .i_osq_nacked               (i_osq_nacked),
        .o_mpsnbuf_pop              (o_mpsnbuf_pop),
        .i_os_rd_req_psn_7_0        (i_os_rdma_rd_resp_psn_7_0),
        .i_os_rd_req_dest_addr      (i_os_rdma_rd_resp_dest_addr),
        .i_os_rd_req_resp_len       (i_os_rdma_rd_resp_length),
        .o_qp_acknack_idx           (o_qp_index_acknack),
        .o_acknack_vld              (o_acknack_valid),
        .o_acknack_psn              (o_acknacked_psn),
        .o_acknack_syndr            (o_acknacked_syndrome),
        .o_acknack_msn              (o_acknacked_msn),
        .o_acknack_opcode           (o_acknacked_opcode),
        .o_qp_timeout_per_qp        (o_qp_timeout_per_qp),
        .rsp_pkt_id                 (rsp_pkt_id),
        .rsp_pkt_type               (rsp_pkt_type),
        .rsp_dest_addr              (rsp_dest_addr),
        .rsp_buf_addr_offset        (rsp_buf_addr_offset),
        .rsp_pkt_len                (rsp_pkt_len),
        .rsp_imm_data               (rsp_imm_data),
        .rsp_tnfr_vld               (rsp_tnfr_vld),
        .rdy_to_rsp_tnfr            (rdy_to_rsp_tnfr),
        .req_tnfr_vld               (req_err_tnfr_vld),
        .rdy_to_req_tnfr            (rdy_to_req_err_tnfr),
        //.ecn_vect                (rx_rsp_ecn_vect),
        .ecn_vect_idx               (rx_rsp_ecn_vect_idx),
        .ecn_vect_vld               (rx_rsp_ecn_update_flg),
        .i_clr_debug_sts            (clr_debug_sts),
        .qp_fatal_rsp_hv_sticky     (debug_sts_reg[0][14]),
        .resp_drop_cnt              (debug_sts_reg[12]),
        .qp_under_recovery_wr_en    (qp_under_recovery_wr_en),
        .qp_under_recovery_wr_idx   (qp_under_recovery_wr_idx),
        .qp_under_recovery_wr_data  (qp_under_recovery_wr_data),
        .init_start                 (init_start),
        .init_count                 (init_count)
    );

    reg_pipe #(
        .C_DATA_WIDTH(C_M_AXI_ADDR_WIDTH + C_RSP_BUF_ADDR_WIDTH + 80)
    ) u_reg_pipe_2 (
        .clk    (core_clk),
        .reset  (core_rst),
        .din    ({rsp_pkt_id, rsp_pkt_type, rsp_dest_addr, rsp_buf_addr_offset, rsp_pkt_len, rsp_imm_data}),
        .vld_in (rsp_tnfr_vld),
        .rdy_out(rdy_to_rsp_tnfr),
        .dout   ({o_rsp_pkt_id, o_rsp_pkt_type, o_rsp_dest_addr, o_rsp_buf_addr_offset, o_rsp_pkt_len, o_rsp_imm_data}),
        .vld_out(o_rsp_tnfr_vld),
        .rdy_in (i_rdy_to_rsp_tnfr)
    );

    rx_rsp_axi_mstr #(
        .NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN(NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN),
        .C_SIM_DEBUG(C_SIM_DEBUG),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .C_RSP_BUF_ADDR_WIDTH(C_RSP_BUF_ADDR_WIDTH)
    ) u_rx_rsp_axi_mstr (
        .clk                          (core_clk),
        .reset                        (core_rst),
        .pkt_id                       (o_rsp_pkt_id),
        .rsp_pkt_type                 (o_rsp_pkt_type),
        .rsp_dest_addr                (o_rsp_dest_addr),
        .rsp_buf_addr_offset          (o_rsp_buf_addr_offset),
        .rsp_pkt_len                  (o_rsp_pkt_len),
        .imm_data                     (o_rsp_imm_data),
        .tnfr_vld                     (o_rsp_tnfr_vld),
        .rdy_to_tnfr                  (i_rdy_to_rsp_tnfr),
        .xrnic_en_pulse               (xrnic_en_pulse),
        .i_err_buf_ba                 (i_rsp_err_pkt_buf_ba),
        .i_err_buf_sz                 (i_rsp_err_buffer_sz),
        .i_err_num_bufs               (i_rsp_err_num_bufs),
        .o_err_buf_ca                 (),
        .o_err_buf_wrptr              (o_rsp_err_buf_wrptr),
        .o_rd_req_pkt_cnt             (),
        .o_rd_rsp_pkt_cnt             (o_inc_rresp_cnt),
        .incr_inv_pkt_cnt             (incr_inv_rsp_pkt_cnt),
        .set_pkt_valdn_err_intr       (set_rsp_pkt_valdn_err_intr),
        .rsp_pkt_buf_addra            (rsp_pkt_buf_addra),
        .rsp_pkt_buf_enb              (rsp_pkt_buf_enb),
        .rsp_pkt_buf_addrb            (rsp_pkt_buf_addrb),
        .rsp_pkt_buf_doutb            (rsp_pkt_buf_doutb),
        .o_rd_rsp_wr_cmpltd           (o_rd_rsp_wr_cmpltd),
        .o_rsp_qp_id                  (o_rsp_qp_id),
        .o_rsp_dest_addr_for_hw_hndshk(o_rsp_dest_addr_for_hw_hndshk),
        .m_axi_awid                   (rx_rsp_axi_awid),
        .m_axi_awaddr                 (rx_rsp_axi_awaddr),
        .m_axi_awlen                  (rx_rsp_axi_awlen),
        .m_axi_awsize                 (rx_rsp_axi_awsize),
        .m_axi_awburst                (rx_rsp_axi_awburst),
        .m_axi_awcache                (rx_rsp_axi_awcache),
        .m_axi_awprot                 (rx_rsp_axi_awprot),
        .m_axi_awvalid                (rx_rsp_axi_awvalid),
        .m_axi_awready                (rx_rsp_axi_awready),
        .m_axi_wdata                  (rx_rsp_axi_wdata),
        .m_axi_wstrb                  (rx_rsp_axi_wstrb),
        .m_axi_wlast                  (rx_rsp_axi_wlast),
        .m_axi_wvalid                 (rx_rsp_axi_wvalid),
        .m_axi_wready                 (rx_rsp_axi_wready),
        .m_axi_awlock                 (rx_rsp_axi_awlock),
        .m_axi_bid                    (rx_rsp_axi_bid),
        .m_axi_bresp                  (rx_rsp_axi_bresp),
        .m_axi_bvalid                 (rx_rsp_axi_bvalid),
        .m_axi_bready                 (rx_rsp_axi_bready),
        .m_axi_arid                   (rx_rsp_axi_arid),
        .m_axi_araddr                 (rx_rsp_axi_araddr),
        .m_axi_arlen                  (rx_rsp_axi_arlen),
        .m_axi_arsize                 (rx_rsp_axi_arsize),
        .m_axi_arburst                (rx_rsp_axi_arburst),
        .m_axi_arcache                (rx_rsp_axi_arcache),
        .m_axi_arprot                 (rx_rsp_axi_arprot),
        .m_axi_arvalid                (rx_rsp_axi_arvalid),
        .m_axi_arready                (rx_rsp_axi_arready),
        .m_axi_rid                    (rx_rsp_axi_rid),
        .m_axi_rdata                  (rx_rsp_axi_rdata),
        .m_axi_rresp                  (rx_rsp_axi_rresp),
        .m_axi_rlast                  (rx_rsp_axi_rlast),
        .m_axi_rvalid                 (rx_rsp_axi_rvalid),
        .m_axi_rready                 (rx_rsp_axi_rready),
        .m_axi_arlock                 (rx_rsp_axi_arlock),

        .free_run_cnt                   (free_run_cnt),
        .i_clr_debug_sts                (clr_debug_sts),
        .axi_awchnl_fifo_unoccupied_r   (debug_sts_reg[7][15:0]),
        .axi_awchnl_max_wait_st         (debug_sts_reg[8][15:0]),
        .axi_wchnl_fifo_unoccupied_r    (debug_sts_reg[7][31:16]),
        .axi_wchnl_max_wait_st          (debug_sts_reg[8][31:16]),
        .axi_awchnl_fifo_empty          (axi_awchnl_fifo_empty),
        .axi_awchnl_fifo_full_sticky    (debug_sts_reg[0][8]),
        .axi_wchnl_fifo_empty           (axi_wchnl_fifo_empty),
        .axi_wchnl_fifo_full_sticky     (debug_sts_reg[0][9]),
        .post_axi_tnfr_fifo_unoccupied_r(debug_sts_reg[5][31:16]),
        .post_axi_tnfr_fifo_empty       (post_axi_tnfr_fifo_empty),
        .post_axi_tnfr_fifo_full_sticky (debug_sts_reg[0][15]),
        .axi_bresp_max_wait_st          (debug_sts_reg[1][31:16])
    );

    assign debug_sts_reg[0][21] = axi_awchnl_fifo_empty;
    assign debug_sts_reg[0][22] = axi_wchnl_fifo_empty;
    assign debug_sts_reg[0][25] = post_axi_tnfr_fifo_empty;

    rx_pkt_intr_ctrl #(
        .C_NUM_QP       (C_NUM_QP),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH)
    ) u_rx_pkt_intr_ctrl (
        .clk                         (core_clk),
        .rst_n                       (core_rst_n),
        .xrnic_enabled               (xrnic_en_pulse),
        .pkt_valdn_err_intr_en       (i_pkt_valdn_err_intr_en),
        .pkt_valdn_err_intr          (o_pkt_valdn_err_intr),
        .pkt_valdn_err_intr_sts      (o_pkt_valdn_err_intr_sts),
        .clr_pkt_valdn_err_intr      (i_clr_pkt_valdn_err_intr),
        .mad_pkt_rcvd_intr_en        (i_mad_pkt_rcvd_intr_en),
        .mad_pkt_rcvd_intr           (o_mad_pkt_rcvd_intr),
        .mad_pkt_rcvd_intr_sts       (o_mad_pkt_rcvd_intr_sts),
        .clr_mad_pkt_rcvd_intr       (i_clr_mad_pkt_rcvd_intr),
        .bypass_pkt_rcvd_intr_en     (i_bypass_pkt_rcvd_intr_en),
        .bypass_pkt_rcvd_intr        (o_bypass_pkt_rcvd_intr),
        .bypass_pkt_rcvd_intr_sts    (o_bypass_pkt_rcvd_intr_sts),
        .clr_bypass_pkt_rcvd_intr    (i_clr_bypass_pkt_rcvd_intr),
        .rnr_nack_gen_intr_en        (i_rnr_nack_gen_intr_en),
        .rnr_nack_gen_intr           (o_rnr_nack_gen_intr),
        .rnr_nack_gen_intr_sts       (o_rnr_nack_gen_intr_sts),
        .clr_rnr_nack_gen_intr       (i_clr_rnr_nack_gen_intr),
        .fatal_err_intr_en           (i_fatal_err_intr_en),
        .fatal_err_intr              (o_fatal_err_intr),
        .fatal_err_intr_sts          (o_fatal_err_intr_sts),
        .clr_fatal_err_intr          (i_clr_fatal_err_intr),
        .qp_pkt_rcvd_intr_en         (i_qp_pkt_rcvd_intr_en),
        .qp_pkt_rcvd_intr            (o_qp_pkt_rcvd_intr),
        //.qp_pkt_rcvd_intr_sts       (o_qp_pkt_rcvd_intr_sts),
        .qp_pkt_rcvd_intr_sts        (o_qp_pkt_rcvd_intr_sts),
        .qp_pkt_rcvd_intr_sts_wr_en  (o_qp_pkt_rcvd_intr_sts_wr_en),
        .qp_pkt_rcvd_intr_sts_wr_addr(o_qp_pkt_rcvd_intr_sts_wr_addr),
        .clr_qp_pkt_rcvd_intr        (i_clr_qp_pkt_rcvd_intr),
        .clr_qp_pkt_rcvd_intr_wr_addr(i_clr_qp_pkt_rcvd_intr_wr_addr),
        //.clr_qp_pkt_rcvd_intr       (i_clr_qp_pkt_rcvd_intr),    
        .set_pkt_valdn_err_intr      (set_pkt_valdn_err_intr),
        .set_mad_pkt_rcvd_intr       (set_mad_pkt_rcvd_intr),
        .set_bypass_pkt_rcvd_intr    (set_bypass_pkt_rcvd_intr),
        .set_rnr_nack_gen_intr       (set_rnr_nack_gen_intr),
        .set_fatal_err_intr          (set_fatal_err_intr),
        .pkt_rcvd_qpid_1             (pkt_rcvd_qpid_1),
        .set_qp_pkt_rcvd_intr_1      (set_qp_pkt_rcvd_intr_1),
        .pkt_rcvd_qpid_2             (pkt_rcvd_qpid_2),
        .set_qp_pkt_rcvd_intr_2      (set_qp_pkt_rcvd_intr_2),
        .cnp_sched_intr_en           (i_cnp_sched_intr_en),
        .set_cnp_sched_intr          (set_cnp_sched_intr),
        .clr_cnp_sched_intr          (i_clr_cnp_sched_intr),
        .cnp_sched_intr_sts          (o_cnp_sched_intr_sts),
        .cnp_sched_intr              (o_cnp_sched_intr),
        .init_start                  (init_start),
        .init_count                  (init_count)
    );

    // =====================================================================
    // Debug Logic
    // =====================================================================

    reg [15:0] req_pkt_buf_remain;
    reg [15:0] req_pkt_buf_unoccupied_min;
    reg [15:0] rsp_pkt_buf_remain;
    reg [15:0] rsp_pkt_buf_unoccupied_min;

    always @(posedge core_clk)
        if (core_rst) begin
            req_pkt_buf_remain         <= {1'b1, {(C_REQ_BUF_ADDR_WIDTH) {1'b0}}};
            req_pkt_buf_unoccupied_min <= {1'b1, {(C_REQ_BUF_ADDR_WIDTH) {1'b0}}};
        end else if (clr_debug_sts) begin
            req_pkt_buf_remain         <= {1'b1, {(C_REQ_BUF_ADDR_WIDTH) {1'b0}}};
            req_pkt_buf_unoccupied_min <= {1'b1, {(C_REQ_BUF_ADDR_WIDTH) {1'b0}}};
        end else begin
            if (req_pkt_buf_addra < req_pkt_buf_addrb) req_pkt_buf_remain <= {1'b1, {(C_REQ_BUF_ADDR_WIDTH) {1'b0}}} - {1'b1, req_pkt_buf_addra} + req_pkt_buf_addrb;
            else req_pkt_buf_remain <= {1'b1, {(C_REQ_BUF_ADDR_WIDTH) {1'b0}}} - req_pkt_buf_addra + req_pkt_buf_addrb;

            if (req_pkt_buf_unoccupied_min > req_pkt_buf_remain) req_pkt_buf_unoccupied_min <= req_pkt_buf_remain;
        end

    always @(posedge core_clk)
        if (core_rst) begin
            rsp_pkt_buf_remain         <= {1'b1, {(C_RSP_BUF_ADDR_WIDTH) {1'b0}}};
            rsp_pkt_buf_unoccupied_min <= {1'b1, {(C_RSP_BUF_ADDR_WIDTH) {1'b0}}};
        end else if (clr_debug_sts) begin
            rsp_pkt_buf_remain         <= {1'b1, {(C_RSP_BUF_ADDR_WIDTH) {1'b0}}};
            rsp_pkt_buf_unoccupied_min <= {1'b1, {(C_RSP_BUF_ADDR_WIDTH) {1'b0}}};
        end else begin
            if (rsp_pkt_buf_addra < rsp_pkt_buf_addrb_with_vld) rsp_pkt_buf_remain <= {1'b1, {(C_RSP_BUF_ADDR_WIDTH) {1'b0}}} - {1'b1, rsp_pkt_buf_addra} + rsp_pkt_buf_addrb_with_vld;
            else rsp_pkt_buf_remain <= {1'b1, {(C_RSP_BUF_ADDR_WIDTH) {1'b0}}} - rsp_pkt_buf_addra + rsp_pkt_buf_addrb_with_vld;

            if (rsp_pkt_buf_unoccupied_min > rsp_pkt_buf_remain) rsp_pkt_buf_unoccupied_min <= rsp_pkt_buf_remain;
        end

    always @(posedge core_clk)
        if (core_rst) begin
            pkt_valdn_err_sticky <= 1'b0;
        end else if (clr_debug_sts) pkt_valdn_err_sticky <= 1'b0;
        else if (set_pkt_valdn_err_intr) pkt_valdn_err_sticky <= 1'b1;

    always @(*) begin
        case (debug_ctrl_in[3:0])
            4'd1:    debug_sts_out = debug_sts_reg[1];
            4'd2:    debug_sts_out = debug_sts_reg[2];
            4'd3:    debug_sts_out = debug_sts_reg[3];
            4'd4:    debug_sts_out = debug_sts_reg[4];
            4'd5:    debug_sts_out = debug_sts_reg[5];
            4'd6:    debug_sts_out = debug_sts_reg[6];
            4'd7:    debug_sts_out = debug_sts_reg[7];
            4'd8:    debug_sts_out = debug_sts_reg[8];
            4'd9:    debug_sts_out = debug_sts_reg[9];
            4'd10:   debug_sts_out = debug_sts_reg[10];
            4'd11:   debug_sts_out = debug_sts_reg[11];
            4'd12:   debug_sts_out = debug_sts_reg[12];
            4'd13:   debug_sts_out = pkt_disc_cnt;
            4'd14:   debug_sts_out = pkt_all_cnt;
            4'd15:   debug_sts_out = {o_rx_rsp_axi_wchnl_wait_st_cnt, o_rx_rsp_axi_wvalid_cnt};
            default: debug_sts_out = debug_sts_reg[0];
        endcase
    end

    assign debug_sts_reg[0][26]    = 1'b0;
    assign debug_sts_reg[0][27]    = 1'b0;
    assign debug_sts_reg[0][30]    = pkt_valdn_err_sticky;
    assign debug_sts_reg[0][31]    = 1'b0;
    assign debug_sts_reg[9][15:0]  = req_pkt_buf_unoccupied_min;
    assign debug_sts_reg[9][31:16] = rsp_pkt_buf_unoccupied_min;

    always @(posedge core_clk)
        if (core_rst) o_rx_rsp_axi_wvalid_cnt <= 16'd0;
        else if (i_global_dbg_cnt_clr || (C_EN_DEBUG_PORTS == 0)) o_rx_rsp_axi_wvalid_cnt <= 16'd0;
        else if (i_global_dbg_cnt_en && rx_rsp_axi_wvalid && (o_rx_rsp_axi_wvalid_cnt < 16'hFFFF)) o_rx_rsp_axi_wvalid_cnt <= o_rx_rsp_axi_wvalid_cnt + 'd1;

    always @(posedge core_clk)
        if (core_rst) o_rx_rsp_axi_wchnl_wait_st_cnt <= 16'd0;
        else if (i_global_dbg_cnt_clr || (C_EN_DEBUG_PORTS == 0)) o_rx_rsp_axi_wchnl_wait_st_cnt <= 16'D0;
        else if (i_global_dbg_cnt_en && rx_rsp_axi_wvalid && !rx_rsp_axi_wready && (o_rx_rsp_axi_wvalid_cnt < 16'hFFFF)) o_rx_rsp_axi_wchnl_wait_st_cnt <= o_rx_rsp_axi_wchnl_wait_st_cnt + 'd1;

endmodule
