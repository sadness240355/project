/* SPDX-License-Identifier: MIT
 * Common register/descriptor definitions for the PCIe SGDMA engine.
 *
 * Software-scheduled multi-queue model:
 *   - BAR0 owns queue contexts addressed as Q_BASE + qid*Q_CONTEXT_SIZE;
 *   - writing Q_DESC_PIDX is the queue doorbell chosen by the driver;
 *   - BAR0 keeps per-queue pending bits and round-robin arbitrates doorbells;
 *   - the scheduler receives operation contexts through a small FIFO;
 *   - each queue uses one descriptor ring and one 32-byte writeback record;
 *     hardware assigns internal per-descriptor MM tags; software keeps one live
 *     operation per queue to protect the single 32-byte WB record.
 */
`ifndef EDEV_SGDMA_DEFS_VH
`define EDEV_SGDMA_DEFS_VH

// Global BAR0 register map
`define EDEV_REG_G_CTRL 16'h0000
`define EDEV_REG_G_STATUS 16'h0004
`define EDEV_REG_G_MODE 16'h0008
`define EDEV_REG_G_IRQ_ENABLE 16'h000c
`define EDEV_REG_G_IRQ_STATUS 16'h0010
`define EDEV_REG_G_ACTIVE_QID 16'h0014
`define EDEV_REG_G_DOORBELL 16'h0018
`define EDEV_REG_G_QUEUE_COUNT 16'h001c
`define EDEV_REG_G_ERROR_CODE 16'h0050
`define EDEV_REG_G_CAP 16'h0060
`define EDEV_REG_G_CONTEXT_SIZE 16'h0064
`define EDEV_REG_G_PERF_DESC_FETCH 16'h0100
`define EDEV_REG_G_PERF_DMA_SUBMIT 16'h0104
`define EDEV_REG_G_PERF_WB_WRITE 16'h0108
`define EDEV_REG_G_PERF_IRQ 16'h010c

// Backward-compatible aliases
`define EDEV_REG_CTRL `EDEV_REG_G_CTRL
`define EDEV_REG_STATUS `EDEV_REG_G_STATUS
`define EDEV_REG_MODE `EDEV_REG_G_MODE
`define EDEV_REG_IRQ_ENABLE `EDEV_REG_G_IRQ_ENABLE
`define EDEV_REG_IRQ_STATUS `EDEV_REG_G_IRQ_STATUS
`define EDEV_REG_ERROR_CODE `EDEV_REG_G_ERROR_CODE
`define EDEV_REG_CAP `EDEV_REG_G_CAP
`define EDEV_REG_PERF_DESC_FETCH `EDEV_REG_G_PERF_DESC_FETCH
`define EDEV_REG_PERF_DMA_SUBMIT `EDEV_REG_G_PERF_DMA_SUBMIT
`define EDEV_REG_PERF_WB_WRITE `EDEV_REG_G_PERF_WB_WRITE
`define EDEV_REG_PERF_IRQ `EDEV_REG_G_PERF_IRQ


// AXI-MM to PCIe requester bridge translation table in BAR0.
// Each entry occupies 0x20 bytes:
//   +0x00 CTRL bit0 enable
//   +0x08 AXI_BASE_LO, +0x0c AXI_BASE_HI
//   +0x10 PCIE_BASE_LO, +0x14 PCIE_BASE_HI
//   +0x18 MASK_LO, +0x1c MASK_HI
`define EDEV_REG_AXI_BR_BASE 16'h0800
`define EDEV_AXI_BR_ENTRY_SIZE 16'h0020
`define EDEV_AXI_BR_CTRL 8'h00
`define EDEV_AXI_BR_AXI_BASE_LO 8'h08
`define EDEV_AXI_BR_AXI_BASE_HI 8'h0c
`define EDEV_AXI_BR_PCIE_BASE_LO 8'h10
`define EDEV_AXI_BR_PCIE_BASE_HI 8'h14
`define EDEV_AXI_BR_MASK_LO 8'h18
`define EDEV_AXI_BR_MASK_HI 8'h1c

// Queue context window: Q_BASE + qid*Q_CONTEXT_SIZE + QREG_*.
`define EDEV_REG_Q_BASE 16'h1000
`define EDEV_Q_CONTEXT_SIZE 16'h0100
`define EDEV_QREG_CTRL 8'h00
`define EDEV_QREG_STATUS 8'h04
`define EDEV_QREG_MODE 8'h08
`define EDEV_QREG_IRQ_ENABLE 8'h0c
`define EDEV_QREG_DESC_BASE_LO 8'h20
`define EDEV_QREG_DESC_BASE_HI 8'h24
`define EDEV_QREG_CMPL_BASE_LO 8'h28
`define EDEV_QREG_CMPL_BASE_HI 8'h2c
`define EDEV_QREG_RING_SIZE 8'h30
`define EDEV_QREG_DESC_PIDX 8'h34
`define EDEV_QREG_DESC_CIDX 8'h38
`define EDEV_QREG_CMPL_PIDX 8'h3c  // legacy/status only; completion ring removed
`define EDEV_QREG_ERROR_CODE 8'h50
`define EDEV_QREG_PERF_DESC_FETCH 8'h60
`define EDEV_QREG_PERF_DMA_SUBMIT 8'h64
`define EDEV_QREG_PERF_WB_WRITE 8'h68
`define EDEV_QREG_PERF_BYTES 8'h6c

// Completion ring was removed; CMPL_BASE points to one 32-byte queue WB record.
`define EDEV_QREG_PERF_CMPL_WRITE `EDEV_QREG_PERF_WB_WRITE

// CTRL bits
`define EDEV_CTRL_ENABLE 0
`define EDEV_CTRL_RESET 1
`define EDEV_CTRL_PAUSE 2
`define EDEV_CTRL_START 3

// Global STATUS bits
`define EDEV_G_STATUS_ENABLED 0
`define EDEV_G_STATUS_BUSY 1
`define EDEV_G_STATUS_SCHED_READY 2
`define EDEV_G_STATUS_DOORBELL_PEND 3
`define EDEV_G_STATUS_FATAL_ERROR 31

// Queue STATUS bits
`define EDEV_STATUS_ENABLED 0
`define EDEV_STATUS_BUSY 1
`define EDEV_STATUS_HALTED 2
`define EDEV_STATUS_RING_EMPTY 3
`define EDEV_STATUS_FATAL_ERROR 31

// Global MODE bits
`define EDEV_MODE_SGDMA 0
`define EDEV_MODE_IRQ 1

// Queue MODE bits: per-queue transfer directions plus fixed queue type.
// Queue type is programmed at queue initialization and is not carried by descriptors.
// 0 = MM queue, 1 = ST/network queue.  MM queues may enable both directions;
// ST queues are direction-exclusive; both H2D/TX and D2H/RX use 16B descriptors.
`define EDEV_QMODE_H2D 0
`define EDEV_QMODE_D2H 1
`define EDEV_QMODE_TYPE_ST 8
`define EDEV_QMODE_BOTH_MASK 32'h0000_0003
`define EDEV_QMODE_TYPE_MASK 32'h0000_0100

// IRQ bits
`define EDEV_IRQ_CMPL 0
`define EDEV_IRQ_ERROR 1
`define EDEV_IRQ_COAL_TIMER 2
`define EDEV_IRQ_CAMERA_FRAME 3

// Descriptor layout/flags
`define EDEV_MM_DESC_SIZE 32
`define EDEV_ST_TX_DESC_SIZE 16
`define EDEV_ST_RX_DESC_SIZE 16
`define EDEV_DESC_VALID 0
`define EDEV_DESC_DIR 1  // 0 = H2D/host-to-device, 1 = D2H/device-to-host
`define EDEV_DESC_IRQ_ON_DONE 2
`define EDEV_DESC_EOP 3
`define EDEV_DESC_ERROR_STOP 5
`define EDEV_DESC_SOP 6
// Descriptor flags are target-agnostic.  Queue type selects MM vs ST.

// 32-byte writeback status bits
`define EDEV_WB_VALID 31
`define EDEV_WB_ERROR 30
`define EDEV_WB_DIR 29
`define EDEV_WB_EOP 28
`define EDEV_WB_SOP 27
`define EDEV_WB_TARGET_ST 26
`define EDEV_WB_TRUNC 25
`define EDEV_WB_DROP 24
`define EDEV_WB_MERGED_ERROR 23

// Hardware ERROR_CODE major classes used by the SGDMA scheduler
`define EDEV_ERR_DESC_FETCH_CPL 8'h01
`define EDEV_ERR_DESC_FETCH_TIMEOUT 8'h02
`define EDEV_ERR_DESC_INVALID 8'h03
`define EDEV_ERR_DESC_LENGTH 8'h04
`define EDEV_ERR_STATUS_FIFO_FULL 8'h05
`define EDEV_ERR_UNEXPECTED_STATUS 8'h06
`define EDEV_ERR_WB_WRITE 8'h07
`define EDEV_ERR_PAYLOAD_READ 8'h10
`define EDEV_ERR_PAYLOAD_WRITE 8'h11
`define EDEV_ERR_QUEUE_DIR 8'h20
`define EDEV_ERR_QID_MISMATCH 8'h21
`define EDEV_ERR_DOORBELL_BUSY 8'h22
`define EDEV_ERR_ST_RX_TRUNC 8'h30
`define EDEV_ERR_ST_RX_BAD_FRAME 8'h31
`define EDEV_ERR_ST_RX_NO_DESC 8'h32
`define EDEV_ERR_ST_TX_UNSUPPORTED 8'h40
`define EDEV_ERR_ST_TX_RC_ERROR 8'h41
`define EDEV_ERR_ST_TX_DISCONTINUE 8'h42
`define EDEV_ERR_ST_TX_BAD_TAG 8'h43
`define EDEV_ERR_ST_TX_DESC 8'h44
`define EDEV_ERR_WB_MERGED_ERROR 8'h52

`endif
