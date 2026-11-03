#ifndef EDEV_CAMERA_IO_H
#define EDEV_CAMERA_IO_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#define CAMERA_DEFAULT_CTRL_DEVICE "/dev/edev0"
#define CAMERA_DEFAULT_QUEUE_DEVICE "/dev/edev0_q64"
#define CAMERA_DEFAULT_QID 64u
#define CAMERA_DEFAULT_CSR_BASE 0x14000ull
#define CAMERA_DEFAULT_RING_SLOTS 32u
#define CAMERA_DEFAULT_TIMEOUT_MS 2000
#define CAMERA_DEFAULT_HOST_BUFFERS 64u
#define CAMERA_DEFAULT_INFLIGHT 16u
#define CAMERA_DEFAULT_DMA_QUEUES 4u
#define CAMERA_MAX_DMA_QUEUES 8u
#define CAMERA_DEFAULT_DISPLAY_HZ 60.0
#define CAMERA_DEFAULT_STRICT_START_TIMEOUT_MS 2000
#define CAMERA_FULL_FRAME_BURST_BUILD_MIN 0x00030021u
#define CAMERA_DMA_BUFFER_INVALID UINT32_MAX
#define CAMERA_DEFAULT_AXIL_CLOCK_HZ 125000000u
#define CAMERA_DEFAULT_SERIAL_BAUD 9600u
#define CAMERA_SERIAL_MIN_BAUD_DIV 8u

/* Camera CSR layout inside the BAR2 page selected by csr_base. */
#define CAMERA_REG_CONTROL        0x00u
#define CAMERA_REG_STATUS         0x04u
#define CAMERA_REG_IRQ_STATUS     0x08u
#define CAMERA_REG_IRQ_ENABLE     0x0cu
#define CAMERA_REG_RING_BASE_LO   0x10u
#define CAMERA_REG_RING_BASE_HI   0x14u
#define CAMERA_REG_FRAME_STRIDE   0x18u
#define CAMERA_REG_RING_SLOTS     0x1cu
#define CAMERA_REG_CONSUMER_IDX   0x20u
#define CAMERA_REG_PRODUCER_IDX   0x24u
#define CAMERA_REG_WIDTH          0x28u
#define CAMERA_REG_HEIGHT         0x2cu
#define CAMERA_REG_FRAME_BYTES    0x30u
#define CAMERA_REG_FRAME_SEQ      0x34u
#define CAMERA_REG_DROP_COUNT     0x38u
#define CAMERA_REG_DIAG_FVAL_EDGES      0x40u
#define CAMERA_REG_DIAG_LVAL_EDGES      0x44u
#define CAMERA_REG_DIAG_DVAL_EDGES      0x48u
#define CAMERA_REG_DIAG_PIX_VALID_WORDS 0x4cu
#define CAMERA_REG_DIAG_BAD_FRAMES      0x50u
#define CAMERA_REG_DIAG_LAST_LINE_BYTES 0x54u
#define CAMERA_REG_EXPECTED_LINE_BYTES  0x58u
#define CAMERA_REG_EXPECTED_FRAME_BYTES 0x5cu
#define CAMERA_REG_DIAG_DROP_FRAME_BYTES 0x60u
#define CAMERA_REG_DIAG_DROP_REASON      0x64u
#define CAMERA_REG_EXPECTED_HEIGHT       0x68u
#define CAMERA_REG_BUILD_ID              0x6cu
#define CAMERA_REG_BUILD_VERSION         0x70u
#define CAMERA_REG_DUP_COMMIT_COUNT      0x74u
#define CAMERA_REG_HLS_STALL_CYCLES      0x78u
#define CAMERA_REG_AXI_STALL_CYCLES      0x7cu
#define CAMERA_REG_CMLK_SYNC_MISMATCH_CYCLES 0xb0u
#define CAMERA_REG_CMLK_SYNC_MISMATCH_EVENTS 0xb4u
#define CAMERA_REG_CMLK_SYNC_MISMATCH_MASK   0xb8u
#define CAMERA_REG_CMLK_ALIGN_LOSS_COUNT       0xbcu
#define CAMERA_REG_HLS_RESYNC_COUNT            0xc0u
#define CAMERA_REG_ISP_ENABLE                  0xc4u
#define CAMERA_REG_ISP_BLACK_LEVEL             0xc8u
#define CAMERA_REG_ISP_WB_GAIN_R               0xccu
#define CAMERA_REG_ISP_WB_GAIN_G               0xd0u
#define CAMERA_REG_ISP_WB_GAIN_B               0xd4u
#define CAMERA_REG_ISP_CCM_00                  0xd8u
#define CAMERA_REG_ISP_CCM_01                  0xdcu
#define CAMERA_REG_ISP_CCM_02                  0xe0u
#define CAMERA_REG_ISP_CCM_10                  0xe4u
#define CAMERA_REG_ISP_CCM_11                  0xe8u
#define CAMERA_REG_ISP_CCM_12                  0xecu
#define CAMERA_REG_ISP_CCM_20                  0xf0u
#define CAMERA_REG_ISP_CCM_21                  0xf4u
#define CAMERA_REG_ISP_CCM_22                  0xf8u
#define CAMERA_ISP_Q12_ONE                     0x1000u
#define CAMERA_ISP_DYNAMIC_VERSION_MIN         0x0003000cu
#define CAMERA_RGB888_BUILD_ID             0x52474233u /* "RGB3" */
/* Build 3.17 stores compact RGB888 end to end; build 3.16 introduced
 * Camera Link 80-bit/1X10_1Y decoding and deterministic frame alignment.
 */
#define CAMERA_RGB888_BUILD_VERSION_MIN    0x00030011u
#define CAMERA_FIFO_SAFE_BUILD_VERSION_MIN  0x00030019u
/* 0x68 carries the expected Bayer frame height. 0x6c..0x7c are
 * read-only build and backpressure diagnostics for the RGB888 HLS pipeline.
 * 0xb0..0xc0 are read-only Camera Link/HLS resynchronization diagnostics.
 * 0xc4..0xf8 are stopped-capture ISP controls. WB gains and signed CCM
 * coefficients use Q4.12 fixed point; build 3.12 or newer is required. */

#define CAMERA_REG_SERIAL_CTRL      0x80u
#define CAMERA_REG_SERIAL_STATUS    0x84u
#define CAMERA_REG_SERIAL_BAUD_DIV  0x88u
#define CAMERA_REG_SERIAL_TXDATA    0x8cu
#define CAMERA_REG_SERIAL_RXDATA    0x90u
#define CAMERA_REG_SERIAL_CC        0x94u
#define CAMERA_REG_SERIAL_INIT_CTRL 0x98u
#define CAMERA_REG_SERIAL_INIT_STAT 0x9cu
#define CAMERA_REG_SERIAL_RX_COUNT  0xa0u
#define CAMERA_REG_SERIAL_TX_COUNT  0xa4u
#define CAMERA_REG_SERIAL_POLARITY  0xa8u
#define CAMERA_REG_SERIAL_CLOCK_HZ  0xacu

#define CAMERA_CTRL_ENABLE        (1u << 0)
#define CAMERA_CTRL_RESET         (1u << 1)
#define CAMERA_CTRL_CLEAR_STATUS  (1u << 2)
#define CAMERA_CTRL_USE_DVAL       (1u << 3)
#define CAMERA_CTRL_OPTION_MASK    (CAMERA_CTRL_USE_DVAL)
#define CAMERA_DEFAULT_BYTES_PER_CLOCK 10u
#define CAMERA_DEFAULT_TAP_ORDER       0x76543210u
#define CAMERA_TAP_ORDER_REVERSE       0x01234567u

#define CAMERA_PROCESS_RAW                     0u
/* FPGA HLS bilinear demosaic path: BayerRG8 input, RGB888 DDR output. */
#define CAMERA_PROCESS_BAYERRG8_RGB888           2u
#define CAMERA_PROCESS_MAX                     CAMERA_PROCESS_BAYERRG8_RGB888

#define CAMERA_STATUS_FIFO_OVERFLOW (1u << 2)
#define CAMERA_STATUS_META_OVERFLOW (1u << 3)
#define CAMERA_STATUS_AXI_ERROR     (1u << 4)
#define CAMERA_STATUS_CFG_ERROR     (1u << 5)
#define CAMERA_STATUS_LINK_READY    (1u << 6)
#define CAMERA_STATUS_FRAME_VALID   (1u << 7)
#define CAMERA_STATUS_CFG_READY     (1u << 8)
#define CAMERA_STATUS_FVAL_SEEN     (1u << 9)
#define CAMERA_STATUS_LVAL_SEEN     (1u << 10)
#define CAMERA_STATUS_DVAL_SEEN     (1u << 11)
#define CAMERA_STATUS_FRAME_SIZE_ERROR (1u << 12)
/* FIFO_OVERFLOW is intentionally diagnostic rather than immediately fatal.
 * New RTL asserts it only after an input word is rejected; the affected frame
 * is then rejected and DROP_COUNT changes.  Older RTL asserted it merely when
 * the FIFO reached full level.  In both cases DROP_COUNT and the hard-error
 * bits below are the reliable software stop conditions.
 */
#define CAMERA_STATUS_ERROR_MASK    (CAMERA_STATUS_META_OVERFLOW | \
                                     CAMERA_STATUS_AXI_ERROR | \
                                     CAMERA_STATUS_CFG_ERROR | \
                                     CAMERA_STATUS_FRAME_SIZE_ERROR)
#define CAMERA_STATUS_DIAG_MASK     (CAMERA_STATUS_FIFO_OVERFLOW)

#define CAMERA_SERIAL_CTRL_RESET       (1u << 0)
#define CAMERA_SERIAL_CTRL_CLEAR       (1u << 4)
#define CAMERA_SERIAL_CTRL_START_INIT  (1u << 8)
#define CAMERA_SERIAL_INIT_START       (1u << 8)
#define CAMERA_SERIAL_INIT_AUTO_ENABLE (1u << 16)

#define CAMERA_SERIAL_STATUS_TX_READY      (1u << 0)
#define CAMERA_SERIAL_STATUS_TX_BUSY       (1u << 1)
#define CAMERA_SERIAL_STATUS_RX_VALID      (1u << 2)
#define CAMERA_SERIAL_STATUS_RX_FULL       (1u << 3)
#define CAMERA_SERIAL_STATUS_RX_OVERFLOW   (1u << 4)
#define CAMERA_SERIAL_STATUS_RX_FRAME_ERR  (1u << 5)
#define CAMERA_SERIAL_STATUS_TX_OVERRUN    (1u << 6)
#define CAMERA_SERIAL_STATUS_INIT_BUSY     (1u << 7)
#define CAMERA_SERIAL_STATUS_INIT_DONE     (1u << 8)
#define CAMERA_SERIAL_STATUS_RX_BUSY       (1u << 9)
#define CAMERA_SERIAL_STATUS_BAUD_DIV_ERR  (1u << 10)
#define CAMERA_SERIAL_RXDATA_VALID         (1u << 8)

#define CAMERA_SERIAL_POLARITY_RX_INVERT       (1u << 0)
#define CAMERA_SERIAL_POLARITY_TX_INVERT       (1u << 1)
#define CAMERA_SERIAL_POLARITY_CC_INVERT_SHIFT 4u
#define CAMERA_SERIAL_POLARITY_CC_INVERT_MASK  (0x0fu << CAMERA_SERIAL_POLARITY_CC_INVERT_SHIFT)
#define CAMERA_SERIAL_POLARITY_RX_RAW_HIGH     (1u << 16)
#define CAMERA_SERIAL_POLARITY_RX_EFF_HIGH     (1u << 17)

struct camera_open_opts {
    const char *ctrl_path;
    const char *queue_path;
    uint64_t csr_base;
    uint64_t ring_base;
    int ring_base_set;      /* ring_base is valid even when it is 0 */
    uint32_t frame_stride;
    int frame_stride_set;   /* frame_stride is valid when nonzero */
    uint32_t ring_slots;
    uint32_t qid;
    int qid_set;
    uint32_t dma_queue_count; /* sync-only driver: number of parallel queue workers; 0=auto */
    int dma_frame_irq;        /* -1=driver/default, 0=poll/no per-frame IRQ, 1=request IRQ completions */
    int driver_frame_irq_dma; /* kernel waits capture IRQ, claims DDR slot, and submits D2H */
    uint32_t irq_enable;
    uint32_t control_flags; /* CAMERA_CTRL_OPTION_MASK bits, e.g. CAMERA_CTRL_USE_DVAL */
    uint32_t bytes_per_clock; /* current RTL accepts only 0 or fixed Deca value 10 */
    uint32_t tap_order;       /* legacy field; ignored only at the fixed default value */
    int tap_order_set;
    uint32_t processing_mode; /* CAMERA_PROCESS_RAW or CAMERA_PROCESS_BAYERRG8_RGB888 */
    uint32_t expected_line_bytes;  /* optional RTL guard; 0 disables equality check */
    uint32_t expected_height;      /* host-visible image height used before first captured frame */
    uint32_t expected_frame_bytes; /* optional RTL guard; 0 disables equality check */
    int enable_capture;
};

struct camera_diag_info {
    uint32_t fval_edges;
    uint32_t lval_edges;
    uint32_t dval_edges;
    uint32_t pix_valid_words;
    uint32_t bad_frames;
    uint32_t last_line_bytes;
    uint32_t drop_frame_bytes;
    uint32_t drop_reason;
};

struct camera_isp_config {
    uint32_t enable_mask;
    uint32_t black_level;
    uint16_t wb_gain[3]; /* R, G, B; unsigned Q4.12 */
    int16_t ccm[9];      /* row-major 3x3; signed Q4.12 */
};

struct camera_csr_snapshot {
    uint32_t control;
    uint32_t status;
    uint32_t producer_idx;
    uint32_t consumer_idx;
    uint32_t ring_fill;
    uint32_t width;
    uint32_t height;
    uint32_t frame_bytes;
    uint32_t frame_seq;
    uint32_t drop_count;
    uint32_t debug_state;
    uint32_t debug_delta;
    uint32_t debug_cdc_flags;
    uint32_t bytes_per_clock;
    uint32_t tap_order;
    uint32_t processing_mode;
    uint32_t build_id;
    uint32_t build_version;
    uint32_t duplicate_commit_count;
    uint32_t hls_stall_cycles;
    uint32_t axi_stall_cycles;
    uint32_t cmlk_sync_mismatch_cycles;
    uint32_t cmlk_sync_mismatch_events;
    uint32_t cmlk_sync_mismatch_mask;
    uint32_t cmlk_align_loss_count;
    uint32_t hls_resync_count;
    uint32_t isp_enable;
    uint32_t isp_black_level;
    uint32_t isp_wb_gain[3];
    int32_t isp_ccm[9];
    struct camera_diag_info diag;
};

struct camera_frame_info {
    uint32_t producer_idx;
    uint32_t consumer_idx;
    uint32_t slot;
    uint32_t width;
    uint32_t height;
    uint32_t frame_bytes;
    uint32_t frame_seq;
    uint32_t drop_count;
    uint32_t status;
    uint32_t dma_buffer_index;
    uint32_t reserved;
    uint64_t dev_addr;
    uint64_t ready_ns; /* Host observation of complete DDR frame; shared by both branches. */
};

struct camera_context;

int camera_open(struct camera_context **out, const struct camera_open_opts *opts);
void camera_close(struct camera_context *ctx);
int camera_reset_capture(struct camera_context *ctx);
int camera_clear_status(struct camera_context *ctx);
int camera_enable_capture(struct camera_context *ctx, uint32_t control_flags);
int camera_disable_capture(struct camera_context *ctx);
int camera_capture_is_enabled(struct camera_context *ctx);
int camera_set_capture_format(struct camera_context *ctx, uint32_t bytes_per_clock,
                              uint32_t tap_order, int tap_order_set);
int camera_refresh_info(struct camera_context *ctx, struct camera_frame_info *info);
int camera_read_diag(struct camera_context *ctx, struct camera_diag_info *diag);
int camera_read_csr_snapshot(struct camera_context *ctx, struct camera_csr_snapshot *snap);
int camera_serial_set_cc(struct camera_context *ctx, uint32_t cc_mask);
int camera_serial_set_polarity(struct camera_context *ctx, uint32_t polarity);
int camera_serial_get_polarity(struct camera_context *ctx, uint32_t *polarity);
int camera_serial_send_command(struct camera_context *ctx, const char *cmd,
                               int timeout_ms, char *response, size_t response_len);
int camera_serial_hw_init(struct camera_context *ctx, int timeout_ms,
                          char *response, size_t response_len);
int camera_set_expected_frame(struct camera_context *ctx, uint32_t line_bytes,
                              uint32_t frame_bytes);
int camera_set_isp_controls(struct camera_context *ctx, uint32_t enable_mask,
                            uint32_t black_level);
int camera_set_isp_config(struct camera_context *ctx,
                          const struct camera_isp_config *config);
int camera_serial_set_baud_div(struct camera_context *ctx, uint32_t baud_div);
int camera_serial_get_baud_div(struct camera_context *ctx, uint32_t *baud_div);
uint32_t camera_serial_status(struct camera_context *ctx);
int camera_set_remote_acquisition_control(struct camera_context *ctx,
                                          const char *start_cmd,
                                          const char *stop_cmd,
                                          int timeout_ms);
int camera_remote_acquisition_start(struct camera_context *ctx);
int camera_remote_acquisition_stop(struct camera_context *ctx);
int camera_wait_ready(struct camera_context *ctx, size_t expected_frame_size,
                      int timeout_ms, struct camera_frame_info *info);
int camera_start_strict(struct camera_context *ctx, uint32_t control_flags,
                        uint32_t expected_line_bytes, uint32_t expected_height,
                        uint32_t expected_frame_bytes, int timeout_ms,
                        struct camera_frame_info *info);
int camera_wait_frame(struct camera_context *ctx, int timeout_ms,
                      struct camera_frame_info *info);
ssize_t camera_read_frame(struct camera_context *ctx,
                          const struct camera_frame_info *info,
                          void *buf, size_t len);
int camera_release_frame(struct camera_context *ctx,
                         const struct camera_frame_info *info);
int camera_dma_open_pool(struct camera_context *ctx, unsigned int buffer_count,
                         size_t buffer_size, unsigned int inflight);
int camera_dma_open_dmabuf_pool(struct camera_context *ctx, const int *dmabuf_fds,
                                unsigned int buffer_count, size_t buffer_size,
                                unsigned int inflight);
int camera_dma_open_dmabuf_pool_strided(struct camera_context *ctx,
                                        const int *dmabuf_fds,
                                        unsigned int buffer_count,
                                        size_t dmabuf_size,
                                        uint32_t dst_buffer_offset,
                                        uint32_t src_line_bytes,
                                        uint32_t dst_line_stride,
                                        uint32_t height,
                                        unsigned int inflight);
int camera_dma_set_latest_mode(struct camera_context *ctx, int enable);
int camera_dma_start_strict(struct camera_context *ctx, uint32_t control_flags,
                            uint32_t expected_line_bytes, uint32_t expected_height,
                            uint32_t expected_frame_bytes, int timeout_ms,
                            struct camera_frame_info *info);
void camera_dma_close_pool(struct camera_context *ctx);
int camera_dma_submit_ready(struct camera_context *ctx);
int camera_dma_reap_frame(struct camera_context *ctx, int timeout_ms,
                          struct camera_frame_info *info,
                          void **data, size_t *len);
int camera_dma_reap_latest_frame(struct camera_context *ctx, int timeout_ms,
                                 struct camera_frame_info *info,
                                 void **data, size_t *len,
                                 uint32_t *host_skipped,
                                 uint32_t *ddr_skipped);
int camera_dma_release_frame(struct camera_context *ctx,
                             const struct camera_frame_info *info);
const char *camera_last_error(const struct camera_context *ctx);

int camera_set_output_mode(struct camera_context *ctx, int raw_bayer);

#endif

