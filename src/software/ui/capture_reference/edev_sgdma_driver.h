/* SPDX-License-Identifier: MIT */
#ifndef EDEV_SGDMA_DRIVER_H
#define EDEV_SGDMA_DRIVER_H

#include <linux/types.h>
#include <linux/ioctl.h>

#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif
#ifndef __packed
#define __packed __attribute__((packed))
#endif
#ifndef __aligned
#define __aligned(x) __attribute__((aligned(x)))
#endif

#define DRIVER_NAME "edev_sgdma"
#define DRIVER_VERSION "9.4-bounce-perf"

#define EDEV_ST_TX_QID_BASE 0u
#define EDEV_ST_TX_QUEUE_COUNT 32u
#define EDEV_ST_RX_QID_BASE 32u
#define EDEV_ST_RX_QUEUE_COUNT 32u
#define EDEV_MM_QID_BASE 64u
#define EDEV_MM_QUEUE_COUNT 8u
#define EDEV_DEFAULT_ST_QUEUE_COUNT (EDEV_ST_TX_QUEUE_COUNT + EDEV_ST_RX_QUEUE_COUNT)
#define EDEV_DEFAULT_MM_QUEUE_COUNT EDEV_MM_QUEUE_COUNT
#define EDEV_DEFAULT_QUEUE_COUNT (EDEV_DEFAULT_ST_QUEUE_COUNT + EDEV_DEFAULT_MM_QUEUE_COUNT)
#define EDEV_MAX_QUEUES 512u
#define EDEV_IRQ_VECTOR_COUNT (EDEV_MM_QUEUE_COUNT + EDEV_ST_TX_QUEUE_COUNT + EDEV_ST_RX_QUEUE_COUNT)
#define EDEV_DEFAULT_RING_SIZE 1024u
#define EDEV_MAX_RING_SIZE 4096u
#define EDEV_DESC_SIZE 32u
#define EDEV_MM_DESC_SIZE 32u
#define EDEV_ST_TX_DESC_SIZE 16u
#define EDEV_ST_RX_DESC_SIZE 16u
#define EDEV_WB_SIZE 32u
#define EDEV_CMPL_SIZE EDEV_WB_SIZE
#define EDEV_MAX_DESC_LEN ((1u << 24) - 1u)
#define EDEV_DEFAULT_FAIR_DESC_BYTES (1u << 20)

/* Global BAR0 registers */
#define EDEV_REG_G_CTRL 0x0000u
#define EDEV_REG_G_STATUS 0x0004u
#define EDEV_REG_G_MODE 0x0008u
#define EDEV_REG_G_IRQ_ENABLE 0x000cu
#define EDEV_REG_G_IRQ_STATUS 0x0010u
#define EDEV_REG_G_ACTIVE_QID 0x0014u
#define EDEV_REG_G_QUEUE_COUNT 0x001cu
#define EDEV_REG_G_ERROR_CODE 0x0050u
#define EDEV_REG_G_CAP 0x0060u
#define EDEV_REG_G_CONTEXT_SIZE 0x0064u
#define EDEV_REG_G_HW_DEPTH 0x0068u

/* Capability bits exposed in EDEV_REG_G_CAP. */
#define EDEV_CAP_SYNC_ONLY BIT(0)
#define EDEV_CAP_SINGLE_WB_MM BIT(1)
#define EDEV_CAP_NO_REAP BIT(2)
#define EDEV_CAP_FAIR_QUEUE BIT(3)
#define EDEV_CAP_CTX_PINGPONG BIT(4)
#define EDEV_CAP_D2H_WB_FENCE BIT(5)
#define EDEV_CAP_AXI_WDATA_FIFO BIT(6)

#define EDEV_REG_Q_BASE 0x1000u
#define EDEV_Q_CONTEXT_SIZE 0x100u
#define EDEV_QREG_CTRL 0x00u
#define EDEV_QREG_STATUS 0x04u
#define EDEV_QREG_MODE 0x08u
#define EDEV_QREG_IRQ_ENABLE 0x0cu
#define EDEV_QREG_DESC_BASE_LO 0x20u
#define EDEV_QREG_DESC_BASE_HI 0x24u
#define EDEV_QREG_CMPL_BASE_LO 0x28u
#define EDEV_QREG_CMPL_BASE_HI 0x2cu
#define EDEV_QREG_RING_SIZE 0x30u
#define EDEV_QREG_DESC_PIDX 0x34u
#define EDEV_QREG_DESC_CIDX 0x38u
#define EDEV_QREG_CMPL_PIDX 0x3cu
#define EDEV_QREG_ERROR_CODE 0x50u
#define EDEV_QREG_PERF_DESC_FETCH 0x60u
#define EDEV_QREG_PERF_DMA_SUBMIT 0x64u
#define EDEV_QREG_PERF_WB_WRITE 0x68u
#define EDEV_QREG_PERF_BYTES 0x6cu
#define EDEV_Q_REG(qid, off) (EDEV_REG_Q_BASE + ((qid) * EDEV_Q_CONTEXT_SIZE) + (off))

#define EDEV_CTRL_ENABLE BIT(0)
#define EDEV_CTRL_RESET BIT(1)
#define EDEV_CTRL_PAUSE BIT(2)
#define EDEV_CTRL_START BIT(3)

#define EDEV_MODE_SGDMA BIT(0)
#define EDEV_MODE_IRQ BIT(1)


#define EDEV_QUEUE_DIR_H2D BIT(0)
#define EDEV_QUEUE_DIR_D2H BIT(1)
#define EDEV_QUEUE_DIR_BOTH (EDEV_QUEUE_DIR_H2D | EDEV_QUEUE_DIR_D2H)
#define EDEV_QUEUE_TYPE_ST BIT(8)
#define EDEV_QUEUE_TYPE_MM 0u
#define EDEV_QUEUE_TYPE_MASK EDEV_QUEUE_TYPE_ST
/* Backward-compatible queue-type alias.  ST queue type is still fixed by qid,
 * but an ST queue is now direction-exclusive: exactly one of H2D/TX or D2H/RX
 * may be enabled in QREG_MODE.  MM queues may still enable both directions.
 */
#define EDEV_QUEUE_ST_ENABLE EDEV_QUEUE_TYPE_ST
#define EDEV_QUEUE_ST_MASK EDEV_QUEUE_TYPE_MASK

#define EDEV_IRQ_CMPL BIT(0)
#define EDEV_IRQ_ERROR BIT(1)
#define EDEV_IRQ_COAL_TIMER BIT(2)
#define EDEV_IRQ_CAMERA_FRAME BIT(3) /* external capture-core frame-ready IRQ */

#define EDEV_G_STATUS_BUSY BIT(1)
#define EDEV_G_STATUS_READY BIT(2)
#define EDEV_G_STATUS_DOORBELL_PENDING BIT(3)
#define EDEV_G_STATUS_FATAL_ERROR BIT(31)

#define EDEV_STATUS_BUSY BIT(1)
#define EDEV_STATUS_FATAL_ERROR BIT(31)

#define EDEV_DESC_VALID BIT(0)
#define EDEV_DESC_DIR_D2H BIT(1)
#define EDEV_DESC_IRQ_ON_DONE BIT(2)
#define EDEV_DESC_EOP BIT(3)
#define EDEV_DESC_ERROR_STOP BIT(5)
#define EDEV_DESC_SOP BIT(6)
/* Descriptor target is inferred from the queue type; no ST target bit is used. */

#define EDEV_WB_STATUS_VALID BIT(31)
#define EDEV_WB_STATUS_ERROR BIT(30)
#define EDEV_WB_STATUS_DIR BIT(29)
#define EDEV_WB_STATUS_EOP BIT(28)
#define EDEV_WB_STATUS_SOP BIT(27)
#define EDEV_WB_STATUS_TARGET_ST BIT(26)
#define EDEV_WB_STATUS_TRUNC BIT(25)
#define EDEV_WB_STATUS_DROP BIT(24)
#define EDEV_WB_STATUS_MERGED_ERROR BIT(23)


/* Hardware/software concurrency capability register.
 * Fair sync RTL keeps one live operation per queue, but accepts multiple queue
 * contexts and time-slices long MM operations across queue dispatches.
 */
#define EDEV_HW_DEPTH_FIELD_MASK 0xffu
#define EDEV_HW_DEPTH_GLOBAL_SHIFT 0u
#define EDEV_HW_DEPTH_MM_SHIFT 8u
#define EDEV_HW_DEPTH_ST_TX_SHIFT 16u
#define EDEV_HW_DEPTH_ST_RX_SHIFT 24u
#define EDEV_HW_DEPTH_DEFAULT 1u
#define EDEV_HW_DEPTH_SYNC_ONLY 0x01010101u
#define EDEV_HW_DEPTH_FAIR_SYNC 0x010101ffu
#define EDEV_HW_DEPTH_FIELD(v, shift) (((v) >> (shift)) & EDEV_HW_DEPTH_FIELD_MASK)

#define EDEV_DMA_DIR_H2D 0u
#define EDEV_DMA_DIR_D2H 1u


/* Submit flags used by EDEV_IOC_SUBMIT_BUFFER.
 * NO_IRQ still produces the queue writeback record, but it does not request
 * a completion interrupt for this operation unless the operation reports an
 * error.  In sync-only mode the submitting thread also polls the coherent WB
 * record as a lost-interrupt fallback, so NO_IRQ is safe for latency/IRQ
 * tuning but does not change completion ordering.
 */
#define EDEV_SUBMIT_F_NO_IRQ (1u << 0)
#define EDEV_SUBMIT_F_SKIP_CPU_SYNC (1u << 1) /* benchmark/platform-coherent only */
#define EDEV_SUBMIT_F_VALID_MASK (EDEV_SUBMIT_F_NO_IRQ | EDEV_SUBMIT_F_SKIP_CPU_SYNC)
/* ST/MM target is inferred from qid.  Submit flags no longer carry target type. */

#define EDEV_ERR_ST_TX_RC_ERROR 0x41u
#define EDEV_ERR_ST_TX_DISCONTINUE 0x42u
#define EDEV_ERR_ST_TX_BAD_TAG 0x43u
#define EDEV_ERR_ST_TX_DESC 0x44u
#define EDEV_ERR_WB_MERGED_ERROR 0x52u

/* MM queue descriptor: 32 bytes, unchanged. */
struct edev_hw_desc
{
    __le64 host_addr;
    __le64 dev_addr;
    __le32 len;
    __le16 tag; /* software diagnostic tag; scheduler allocates internal MM descriptor tags */
    __le16 flags;
    __le32 seq;
    __le32 rsvd;
} __packed __aligned(32);

/* ST H2D/TX queue descriptor: 16 bytes.  The queue consumes descriptors at
 * base + pidx*16.  host_addr points to the host payload, len is the byte count,
 * tag is a software diagnostic tag copied into completion WB, and qid is used as a
 * hardware-side consistency check against the queue context.
 */
struct edev_st_tx_desc
{
    __le64 host_addr;
    __le32 len;
    __le16 tag;
    __le16 qid;
} __packed __aligned(16);

/* ST D2H/RX queue descriptor: 16 bytes.  The queue consumes descriptors at
 * base + pidx*16.  host_addr points to a driver-owned or userspace-registered
 * receive buffer, len is the maximum writable byte count for this descriptor,
 * tag is copied into the completion WB, and qid is a hardware-side consistency
 * check against the queue context.  The actual packet byte count is still
 * reported through the completion bytes field.
 */
struct edev_st_rx_desc
{
    __le64 host_addr;
    __le32 len;
    __le16 tag;
    __le16 qid;
} __packed __aligned(16);

/* 32-byte writeback record.  Sync-only ST and MM queues both use one record. */
struct edev_hw_wb
{
    __le32 status;
    __le32 completed_cidx;
    __le32 submitted_cidx;
    __le32 bytes;
    __le32 error_code;
    __le32 qid;
    __le32 swtag; /* software diagnostic tag */
    __le32 user;  /* ST_RX: tuser snapshot; MM mode: reserved */
} __packed __aligned(32);


struct edev_ioc_hw_caps
{
    __u32 cap;                    /* raw EDEV_REG_G_CAP */
    __u32 queue_count;            /* enabled software queue count */
    __u32 ring_size;              /* descriptor ring entries per queue */
    __u32 hw_depth_raw;           /* raw EDEV_REG_G_HW_DEPTH */
    __u32 hw_global_outstanding;  /* hardware global live-op capacity */
    __u32 hw_mm_outstanding;      /* hardware MM live-op recommendation */
    __u32 hw_st_tx_outstanding;   /* hardware ST TX/H2D live-op recommendation */
    __u32 hw_st_rx_outstanding;   /* hardware ST RX/D2H live-op recommendation */
    __u32 sw_global_outstanding;  /* effective fair queue capacity, capped at 255 */
    __u32 sw_global_live;         /* current busy queues in sync/fair mode */
    __u32 mm_queue_limit;         /* one live operation per MM queue */
    __u32 st_queue_limit;         /* one live operation per ST queue */
    __u32 ioctl_batch_limit;      /* maximum sequential SUBMIT_BATCH entries */
    __u32 reserved[3];
};

struct edev_ioc_queue_cfg
{
    __u32 qid;
    __u32 dir_mask; /* EDEV_QUEUE_DIR_H2D/D2H/BOTH */
    __u32 ring_size;
    __u32 flags; /* EDEV_QUEUE_TYPE_ST is read-only queue type; EDEV_IOC_QUEUE_CONFIG cannot change it */
};

struct edev_ioc_queue_stats
{
    __u32 qid;
    __u32 status;
    __u32 error_code;
    __u32 desc_pidx;
    __u32 desc_cidx;
    __u32 cmpl_pidx; /* hardware/status register; sync-only queues use one WB record */
    __u32 cmpl_cidx; /* last completed descriptor index */
    __u32 sw_desc_used;
    __u32 sw_cmpl_pending;
    __u32 sw_cmpl_done;
    __u32 perf_desc_fetch;
    __u32 perf_dma_submit;
    __u32 perf_wb_write;
    __u32 perf_bytes;
};

struct edev_ioc_user_buffer
{
    __u64 user_va;
    __u64 size;
    __u32 dir;
    __u32 buf_id;
    __u32 nmap;
    __u32 reserved;
};

struct edev_ioc_dma_submit
{
    __u32 qid;
    __u32 buf_id;
    __u32 dir;
    __u32 flags;
    __u64 buf_offset;
    __u64 len;
    __u64 dev_addr;
    __u64 op_id;      /* returned by EDEV_IOC_SUBMIT_BUFFER */
    __u32 desc_start; /* returned descriptor-ring start index */
    __u32 desc_count; /* returned descriptor count */
    __s32 status;     /* sync-only: final DMA completion status returned by submit */
    __u32 reserved;
};

/* Compatibility completion record from the former standalone NIC driver.
 * Unified submits are synchronous, so REAP returns the queue's cached result. */
struct edev_ioc_dma_cmpl
{
    __u32 qid;
    __u32 flags;
    __u64 timeout_ms;
    __u64 op_id;
    __u64 bytes_done;
    __s32 status;
    __u32 error_code;
    __u32 desc_cidx;
    __u32 submitted_cidx;
    __u32 desc_count;
    __u32 swtag;
};

/* 2D D2H submit for camera display surfaces.
 * Each row copies row_bytes from device DDR at dev_addr + y * dev_stride into
 * the registered/imported buffer at buf_offset + y * buf_stride.  It is used by
 * the Orin NvBufSurface path when the visible RGBA line is smaller than the
 * exported DMA-BUF allocation span or when the destination surface has pitch.
 */
struct edev_ioc_dma_submit_2d
{
    __u32 qid;
    __u32 buf_id;
    __u32 dir;
    __u32 flags;
    __u64 buf_offset;
    __u64 dev_addr;
    __u32 row_bytes;
    __u32 rows;
    __u32 dev_stride;
    __u32 buf_stride;
    __u64 op_id;
    __u32 desc_start;
    __u32 desc_count;
    __u64 bytes_done;
    __s32 status;
    __u32 reserved[3];
};

#define EDEV_BATCH_MAX 64u
#define EDEV_BATCH_F_IRQ_LAST (1u << 0)

/* Batch descriptors point to userspace arrays.  In sync-only mode SUBMIT_BATCH
 * executes entries in order and each entry is complete when the ioctl returns.
 */
struct edev_ioc_submit_batch
{
    __u64 entries; /* userspace pointer to struct edev_ioc_dma_submit[count] */
    __u32 count;
    __u32 flags; /* EDEV_BATCH_F_* */
    __u32 submitted;
    __s32 status;
};

struct edev_ioc_reap_batch
{
    __u64 entries;
    __u32 count;
    __u32 flags;
    __u32 completed;
    __s32 status;
    __u64 timeout_ms;
};

/* Non-zero-copy/bounce submit: user buffer is copied through a driver-owned
 * DMA bounce buffer before/after the actual DMA.  bytes_done and status report
 * the end-to-end result.  dma_ns covers hardware submit+WB wait time inside the
 * driver, while copy_ns covers copy_from_user/copy_to_user time.
 */
struct edev_ioc_bounce_submit
{
    __u32 qid;
    __u32 dir;
    __u32 flags; /* EDEV_SUBMIT_F_NO_IRQ is honored; SKIP_CPU_SYNC is implied */
    __u32 reserved0;
    __u64 user_va;
    __u64 len;
    __u64 dev_addr;
    __u64 op_id;
    __u64 bytes_done;
    __u64 dma_ns;
    __u64 copy_ns;
    __u32 desc_count;
    __s32 status;
    __u32 reserved1[2];
};

/* Import a dma-buf FD (for DRM/NvBufSurface/other exporter) as a DMA target.
 * The returned buf_id can be used with EDEV_IOC_SUBMIT_BUFFER exactly like a
 * userspace-registered buffer.  D2H imports are intended for direct scanout or
 * GPU-visible buffers.
 */
struct edev_ioc_dmabuf
{
    __s32 fd;
    __u32 dir;
    __u64 size;
    __u64 offset;
    __u32 buf_id;
    __u32 nmap;
    __u32 reserved[4];
};

/* Wait until the camera capture block has at least one DDR frame available.
 * The interrupt source is the camera frame-ready pulse routed into the PCIe
 * MSI-X path; polling is only a fallback if IRQ mode is unavailable.
 */

/* Kernel-assisted camera-frame D2H.  The driver waits for the camera capture
 * block's frame-ready MSI-X pulse, claims the next DDR ring slot by advancing
 * the capture consumer index, and immediately runs a synchronous D2H DMA into
 * the supplied registered user buffer or imported dma-buf.
 */
struct edev_ioc_camera_submit_frame
{
    __u64 csr_base;
    __u64 ring_base;
    __u32 frame_stride;
    __u32 ring_slots;
    __u32 qid;
    __u32 buf_id;
    __u32 flags;
    __u32 timeout_ms;
    __u64 len;
    __u64 buf_offset;
    __u64 dev_addr;
    __u64 op_id;
    __u32 producer_idx;
    __u32 consumer_idx;
    __u32 slot;
    __u32 width;
    __u32 height;
    __u32 frame_bytes;
    __u32 frame_seq;
    __u32 drop_count;
    __u32 status;
    __u32 desc_count;
    __s32 result;
    __u32 reserved[3];
};
struct edev_ioc_camera_wait
{
    __u64 csr_base;
    __u32 timeout_ms;
    __u32 flags;
    __u32 producer_idx;
    __u32 consumer_idx;
    __u32 status;
    __u32 width;
    __u32 height;
    __u32 frame_bytes;
    __u32 drop_count;
    __s32 result;
};

#define EDEV_IOC_MAGIC 'E'
#define EDEV_IOC_SET_IRQ_MODE _IOW(EDEV_IOC_MAGIC, 1, __u32)
#define EDEV_IOC_RESET_ENGINE _IO(EDEV_IOC_MAGIC, 4)
#define EDEV_IOC_QUEUE_CONFIG _IOW(EDEV_IOC_MAGIC, 20, struct edev_ioc_queue_cfg)
#define EDEV_IOC_QUEUE_STATS _IOWR(EDEV_IOC_MAGIC, 21, struct edev_ioc_queue_stats)
#define EDEV_IOC_REGISTER_BUFFER _IOWR(EDEV_IOC_MAGIC, 30, struct edev_ioc_user_buffer)
#define EDEV_IOC_UNREGISTER_BUFFER _IOW(EDEV_IOC_MAGIC, 31, __u32)
#define EDEV_IOC_SUBMIT_BUFFER _IOWR(EDEV_IOC_MAGIC, 32, struct edev_ioc_dma_submit)
#define EDEV_IOC_REAP_CMPL _IOWR(EDEV_IOC_MAGIC, 33, struct edev_ioc_dma_cmpl)
#define EDEV_IOC_SUBMIT_BATCH _IOWR(EDEV_IOC_MAGIC, 34, struct edev_ioc_submit_batch)
#define EDEV_IOC_REAP_BATCH _IOWR(EDEV_IOC_MAGIC, 35, struct edev_ioc_reap_batch)
#define EDEV_IOC_GET_CAPS _IOR(EDEV_IOC_MAGIC, 36, struct edev_ioc_hw_caps)
#define EDEV_IOC_SUBMIT_BOUNCE _IOWR(EDEV_IOC_MAGIC, 37, struct edev_ioc_bounce_submit)
#define EDEV_IOC_REGISTER_DMABUF _IOWR(EDEV_IOC_MAGIC, 38, struct edev_ioc_dmabuf)
#define EDEV_IOC_CAMERA_WAIT_FRAME _IOWR(EDEV_IOC_MAGIC, 39, struct edev_ioc_camera_wait)
#define EDEV_IOC_CAMERA_SUBMIT_FRAME _IOWR(EDEV_IOC_MAGIC, 40, struct edev_ioc_camera_submit_frame)
#define EDEV_IOC_SUBMIT_2D _IOWR(EDEV_IOC_MAGIC, 41, struct edev_ioc_dma_submit_2d)


#ifndef EDEV_IOC_CAMERA_D2H_FRAME
#define EDEV_IOC_CAMERA_D2H_FRAME EDEV_IOC_CAMERA_SUBMIT_FRAME
#define edev_ioc_camera_dma_frame edev_ioc_camera_submit_frame
#endif

/* Compatibility ABI for coherent buffers and direct BAR2 access. */
#define RECONIC_DMA_IOC_MAGIC 'R'
#define RECONIC_DMA_IOCTL_MAGIC RECONIC_DMA_IOC_MAGIC
struct reconic_dma_alloc_req {
    __u64 size;
    __u64 dma_addr;
    __u64 mmap_offset;
};
struct reconic_bar2_access {
    __u32 offset;
    __u32 value;
};
#define RECONIC_DMA_IOC_ALLOC_COHERENT \
    _IOWR(RECONIC_DMA_IOC_MAGIC, 1, struct reconic_dma_alloc_req)
#define RECONIC_DMA_IOC_FREE_COHERENT _IO(RECONIC_DMA_IOC_MAGIC, 2)
#define RECONIC_DMA_IOC_BAR2_READ \
    _IOWR(RECONIC_DMA_IOC_MAGIC, 3, struct reconic_bar2_access)
#define RECONIC_DMA_IOC_BAR2_WRITE \
    _IOW(RECONIC_DMA_IOC_MAGIC, 4, struct reconic_bar2_access)
#define RECONIC_DMA_IOC_ALLOC_COHERENT_LEGACY \
    _IOWR(RECONIC_DMA_IOC_MAGIC, 2, struct reconic_dma_alloc_req)
#define RECONIC_DMA_IOC_FREE_COHERENT_LEGACY _IO(RECONIC_DMA_IOC_MAGIC, 3)

#ifdef __KERNEL__
struct edev_dev;

struct edev_kernel_cmpl {
    __u32 status;
    __u32 error_code;
    __u32 bytes;
    __u32 user;
};

int edev_kernel_async_submit(struct edev_dev *edev, __u32 qid,
                             dma_addr_t host_addr, __u32 len, bool h2d);
int edev_kernel_async_poll(struct edev_dev *edev, __u32 qid,
                           struct edev_kernel_cmpl *cmpl);
void edev_kernel_async_cancel(struct edev_dev *edev, __u32 qid);
__u32 edev_kernel_st_tx_count(struct edev_dev *edev);
__u32 edev_kernel_st_rx_base(struct edev_dev *edev);
__u32 edev_kernel_st_rx_count(struct edev_dev *edev);
int edev_bar2_write32(struct edev_dev *edev, __u32 offset, __u32 value);
int edev_bar2_read32(struct edev_dev *edev, __u32 offset, __u32 *value);
#endif

#endif
