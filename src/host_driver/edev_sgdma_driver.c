// SPDX-License-Identifier: MIT
/*
 * Synchronous single-live-op PCIe SGDMA driver.
 *
 * The v8 driver removes the userspace-visible asynchronous operation model.
 * EDEV_IOC_SUBMIT_BUFFER returns only after the corresponding DMA operation has
 * completed and the single queue writeback record has been consumed.
 * EDEV_IOC_SUBMIT_BATCH is retained as a sequential compatibility wrapper, not
 * as a fused performance path.  Legacy asynchronous REAP ioctls are removed from
 * this low-bubble sync-only ABI drop.
 *
 * Fairness update: software may submit one synchronous operation per queue in
 * parallel.  Normal submit no longer holds the global scheduler mutex while it
 * waits for completion; BAR0/RTL performs fair queue requeueing at descriptor
 * quantum boundaries.  MM descriptors are capped by fair_desc_bytes so a single
 * payload descriptor cannot monopolize the hardware datapath.
 *
 * The v8.2 RTL scheduler adds a ping-pong preload bank for the next queue
 * context.  No software asynchronous completion tracking is reintroduced for
 * this context-switch optimization.
 *
 * The v8.6 RTL disables the high-tag D2H readback fence by default and adds
 * a BRAM-backed H2D AXI-W FIFO.  D2H relies on ordered posted writes for WB
 * visibility, while H2D no longer backpressures PCIe RC on short WREADY dips.
 */

#include <linux/bits.h>
#include "edev_sgdma_driver.h"
#include "reconic_netdev.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/log2.h>
#include <linux/idr.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/rwsem.h>
#include <linux/list.h>
#include <linux/atomic.h>
#include <linux/sched/signal.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0)
#include <linux/pci.h>
#endif

#ifndef PCI_EXP_DEVCAP_EXT_TAG
#define PCI_EXP_DEVCAP_EXT_TAG 0x20
#endif
#ifndef PCI_EXP_DEVCTL_EXT_TAG
#define PCI_EXP_DEVCTL_EXT_TAG 0x0100
#endif
#ifndef PCI_VENDOR_ID_NVIDIA
#define PCI_VENDOR_ID_NVIDIA 0x10de
#endif

#define EDEV_MAX_DEVICES 8u
#define EDEV_MINOR_STRIDE (EDEV_MAX_QUEUES + 1u)
#define EDEV_DEFAULT_RW_CHUNK_BYTES (16ul << 20)
#define EDEV_DEFAULT_BOUNCE_BYTES (16ul << 20)
#define EDEV_SYNC_POLL_US_DEFAULT 25u

MODULE_DESCRIPTION("unified ReconIC netdev and camera SGDMA PCIe driver");
MODULE_AUTHOR("OpenAI / PCIE project adaptation");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_VERSION(DRIVER_VERSION);

static unsigned int queue_count = EDEV_DEFAULT_QUEUE_COUNT;
module_param(queue_count, uint, 0444);
MODULE_PARM_DESC(queue_count, "total DMA queues");

static unsigned int st_queue_count = EDEV_DEFAULT_ST_QUEUE_COUNT;
module_param(st_queue_count, uint, 0444);
MODULE_PARM_DESC(st_queue_count, "number of initial fixed ST queues");

static unsigned int st_h2d_queue_count = EDEV_ST_TX_QUEUE_COUNT;
module_param(st_h2d_queue_count, uint, 0444);
MODULE_PARM_DESC(st_h2d_queue_count, "number of initial ST queues configured as H2D/TX-only");

static unsigned int ring_size = EDEV_DEFAULT_RING_SIZE;
module_param(ring_size, uint, 0444);
MODULE_PARM_DESC(ring_size, "descriptor ring entries per queue");

static unsigned int irq_mode = 0;
module_param(irq_mode, uint, 0644);
MODULE_PARM_DESC(irq_mode, "enable DMA completion interrupts; keep 0 for polling-only DMA completion/recovery");

static unsigned int camera_irq_mode = 1;
module_param(camera_irq_mode, uint, 0644);
MODULE_PARM_DESC(camera_irq_mode, "enable capture-core frame-ready MSI-X interrupts independent of DMA completion IRQs");

static unsigned int irq_vectors = 1;
module_param(irq_vectors, uint, 0444);
MODULE_PARM_DESC(irq_vectors, "MSI-X vectors to request; 1 for shared vector or queue_count for legacy per-queue vectors");

static bool irq_fallback_poll = true;
module_param(irq_fallback_poll, bool, 0444);
MODULE_PARM_DESC(irq_fallback_poll, "fall back to polling if MSI-X allocation is unavailable");

static unsigned int shared_irq_vector = 1;
module_param(shared_irq_vector, uint, 0644);
MODULE_PARM_DESC(shared_irq_vector, "map all queue IRQs to MSI-X vector 0");

static unsigned int max_registered_buffers = 64;
module_param(max_registered_buffers, uint, 0644);
MODULE_PARM_DESC(max_registered_buffers, "maximum registered user DMA buffers per open file; 0 disables this software cap");

static unsigned long max_registered_bytes = 512ul << 20;
module_param(max_registered_bytes, ulong, 0644);
MODULE_PARM_DESC(max_registered_bytes, "maximum size of one registered user DMA buffer; 0 disables this software cap");

static unsigned long rw_chunk_bytes = EDEV_DEFAULT_RW_CHUNK_BYTES;
module_param(rw_chunk_bytes, ulong, 0644);
MODULE_PARM_DESC(rw_chunk_bytes, "maximum bytes per internal read/write DMA chunk");

static unsigned long bounce_chunk_bytes = EDEV_DEFAULT_BOUNCE_BYTES;
module_param(bounce_chunk_bytes, ulong, 0644);
MODULE_PARM_DESC(bounce_chunk_bytes, "maximum bytes per non-zero-copy bounce DMA chunk; buffers are allocated lazily per queue");

static unsigned int ioctl_batch_limit = EDEV_BATCH_MAX;
module_param(ioctl_batch_limit, uint, 0644);
MODULE_PARM_DESC(ioctl_batch_limit, "maximum sequential entries accepted by SUBMIT_BATCH");

static unsigned long dev_mem_size = 0;
module_param(dev_mem_size, ulong, 0444);
MODULE_PARM_DESC(dev_mem_size, "optional device memory aperture size for read/write offset bounds; 0 disables bounds checking");

static unsigned int sync_poll_us = EDEV_SYNC_POLL_US_DEFAULT;
module_param(sync_poll_us, uint, 0644);
MODULE_PARM_DESC(sync_poll_us, "coherent WB polling interval in microseconds when IRQ mode is disabled");

static unsigned int sync_busy_poll_us = 200;
module_param(sync_busy_poll_us, uint, 0644);
MODULE_PARM_DESC(sync_busy_poll_us, "short busy-poll window before IRQ/sleep wait; default 200us with irq_mode=1");

static unsigned int sync_timeout_ms = 10000;
module_param(sync_timeout_ms, uint, 0644);
MODULE_PARM_DESC(sync_timeout_ms, "synchronous DMA completion timeout in milliseconds; 0 waits forever");

static unsigned long fair_desc_bytes = EDEV_DEFAULT_FAIR_DESC_BYTES;
module_param(fair_desc_bytes, ulong, 0644);
MODULE_PARM_DESC(fair_desc_bytes, "maximum bytes per hardware descriptor for fair queue scheduling; 0 uses 1 MiB");

static unsigned int pcie_mrrs_bytes = 512;
module_param(pcie_mrrs_bytes, uint, 0644);
MODULE_PARM_DESC(pcie_mrrs_bytes, "PCIe MRRS in bytes; 0=auto, 256 on NVIDIA/Tegra root ports, 512 otherwise");

#define EDEV_CAMERA_DEFAULT_CSR_BASE   0x14000u

static unsigned int camera_csr_base = EDEV_CAMERA_DEFAULT_CSR_BASE;
module_param(camera_csr_base, uint, 0644);
MODULE_PARM_DESC(camera_csr_base, "BAR2 offset of camera capture CSR block for frame-ready wait");


#define EDEV_CAMERA_REG_STATUS         0x04u
#define EDEV_CAMERA_REG_RING_BASE_LO   0x10u
#define EDEV_CAMERA_REG_FRAME_STRIDE   0x18u
#define EDEV_CAMERA_REG_RING_SLOTS     0x1cu
#define EDEV_CAMERA_REG_PRODUCER_IDX   0x24u
#define EDEV_CAMERA_REG_CONSUMER_IDX   0x20u
#define EDEV_CAMERA_REG_WIDTH          0x28u
#define EDEV_CAMERA_REG_HEIGHT         0x2cu
#define EDEV_CAMERA_REG_FRAME_BYTES    0x30u
#define EDEV_CAMERA_REG_DROP_COUNT     0x38u
#define EDEV_CAMERA_REG_IRQ_STATUS     0x08u
#define EDEV_CAMERA_REG_IRQ_ENABLE     0x0cu
#define EDEV_CAMERA_IRQ_FRAME_READY    0x00000001u
/* Bit 2 is the capture FIFO-overflow diagnostic.  Do not treat it as a
 * frame-ready wait fatal condition here: software validates frame metadata and
 * drop counters after claiming each DDR slot, and older RTL can assert bit 2
 * transiently around reset/enable without losing the published frame.
 */
#define EDEV_CAMERA_STATUS_ERROR_MASK  ((1u << 3) | (1u << 4) | (1u << 5) | (1u << 12))

struct edev_dev;
struct edev_file_ctx;
struct edev_user_buffer;

struct edev_cmpl_cache {
    u64 op_id;
    u64 bytes_done;
    s32 status;
    u32 flags;
    u32 error_code;
    u32 desc_cidx;
    u32 submitted_cidx;
    u32 desc_count;
    u32 swtag;
};

struct edev_queue {
    void *ring_mem;
    size_t ring_mem_size;
    dma_addr_t ring_dma;

    struct edev_hw_desc *desc;
    dma_addr_t desc_dma;
    struct edev_hw_wb *cmpl;
    dma_addr_t cmpl_dma;

    void *bounce_cpu;
    dma_addr_t bounce_dma;
    size_t bounce_size;
    struct sg_table bounce_sgt;
    bool bounce_sgt_valid;
    struct mutex bounce_lock;

    struct edev_dev *edev;
    u32 qid;
    u32 dir_mask;
    u32 ring_size;
    u32 ring_mask;

    u32 sw_pidx;
    u32 sw_cidx;
    u32 last_error;
    struct edev_cmpl_cache last_cmpl;

    bool configured;
    bool busy;
    bool kernel_async;
    u64 kernel_op_id;
    u32 kernel_target_cidx;
    u32 kernel_desc_count;
    u16 kernel_swtag;
    int kernel_recovery_epoch;
    struct mutex lock;
    wait_queue_head_t waitq;
};

struct edev_irq_vector {
    struct edev_dev *edev;
    u32 vector;
};

struct edev_dev {
    struct pci_dev *pdev;
    struct device *dev;
    void __iomem *bar[6];
    resource_size_t bar_len[6];
    struct cdev cdev;
    dev_t devt;
    u32 instance_id;
    u32 devt_count;
    struct device *chardev;
    struct device **queue_chardevs;
    struct edev_queue *queues;
    u32 queue_count;
    bool irq_mode;
    bool camera_irq_mode;
    u32 hw_cap;
    u32 hw_depth_raw;
    u32 hw_global_outstanding_limit;
    u32 hw_mm_outstanding_limit;
    u32 hw_st_tx_outstanding_limit;
    u32 hw_st_rx_outstanding_limit;
    struct mutex sched_lock;
    struct rw_semaphore submit_rwsem;
    wait_queue_head_t waitq;
    wait_queue_head_t camera_waitq;
    struct mutex camera_lock;
    atomic64_t camera_irq_count;
    atomic64_t camera_wait_wake_count;
    atomic64_t next_op_id;
    atomic_t recovery_epoch;
    bool hw_offline;
    int irq_vector_count;
    struct edev_irq_vector *irq_vec;
    struct reconic_netdev *netdev;
};

struct edev_user_buffer {
    struct list_head node;
    u32 id;
    bool h2d;
    unsigned long user_va;
    size_t size;
    int npages;
    int nmap;
    unsigned int active;
    struct page **pages;
    struct sg_table sgt;
    enum dma_data_direction dma_dir;
    bool imported_dmabuf;
    struct dma_buf *dmabuf;
    struct dma_buf_attachment *dmabuf_attach;
    struct sg_table *dmabuf_sgt;
};

struct edev_file_ctx {
    struct edev_dev *edev;
    bool queue_file;
    u32 qid;
    struct mutex buf_lock;
    struct list_head buffers;
    u32 next_buf_id;
    u32 buffer_count;
    void *coh_cpu;
    dma_addr_t coh_dma;
    size_t coh_size;
};

static dev_t edev_devt_base;
static struct class *edev_class;
static DEFINE_IDA(edev_ida);

static inline void edev_raw_write32(struct edev_dev *edev, u32 reg, u32 val)
{
    iowrite32(val, edev->bar[0] + reg);
}

static inline u32 edev_raw_read32(struct edev_dev *edev, u32 reg)
{
    return ioread32(edev->bar[0] + reg);
}

static void edev_mark_hw_offline(struct edev_dev *edev, u32 reg)
{
    if (!READ_ONCE(edev->hw_offline)) {
        WRITE_ONCE(edev->hw_offline, true);
        dev_err(edev->dev,
                "BAR0 read reg=0x%04x returned 0xffffffff; SGDMA MMIO disabled until driver reload\n",
                reg);
    }
}

static inline void edev_write32(struct edev_dev *edev, u32 reg, u32 val)
{
    if (READ_ONCE(edev->hw_offline) || !edev->bar[0])
        return;
    edev_raw_write32(edev, reg, val);
}

static inline u32 edev_read32(struct edev_dev *edev, u32 reg)
{
    u32 val;

    if (READ_ONCE(edev->hw_offline) || !edev->bar[0])
        return 0xffffffffu;

    val = edev_raw_read32(edev, reg);
    if (val == 0xffffffffu)
        edev_mark_hw_offline(edev, reg);
    return val;
}

int edev_bar2_write32(struct edev_dev *edev, u32 offset, u32 value)
{
    if (!edev || !edev->bar[2] || edev->bar_len[2] < sizeof(u32) ||
        offset > edev->bar_len[2] - sizeof(u32))
        return -EINVAL;
    iowrite32(value, edev->bar[2] + offset);
    (void)ioread32(edev->bar[2] + offset);
    return 0;
}

int edev_bar2_read32(struct edev_dev *edev, u32 offset, u32 *value)
{
    if (!edev || !value || !edev->bar[2] ||
        edev->bar_len[2] < sizeof(u32) ||
        offset > edev->bar_len[2] - sizeof(u32))
        return -EINVAL;
    if (pci_channel_offline(edev->pdev))
        return -EIO;
    *value = ioread32(edev->bar[2] + offset);
    /* 0xffffffff is valid specifically for shell status (offset 0x10): the
     * unused reset-done bits are tied high, so all completed resets can make
     * this register all ones.  Keep the legacy all-ones failure detection for
     * other BAR2 registers, where it remains useful for an AXI decode failure.
     */
    return *value == 0xffffffffu && offset != 0x0010u ? -EIO : 0;
}

static inline void edev_q_write32(struct edev_dev *edev, u32 qid, u32 off, u32 val)
{
    edev_write32(edev, EDEV_Q_REG(qid, off), val);
}

static inline u32 edev_q_read32(struct edev_dev *edev, u32 qid, u32 off)
{
    return edev_read32(edev, EDEV_Q_REG(qid, off));
}

static u32 ring_next(struct edev_queue *q, u32 idx)
{
    return (idx == q->ring_size - 1) ? 0 : idx + 1;
}

static u32 ring_distance(struct edev_queue *q, u32 from, u32 to)
{
    return (to - from) & q->ring_mask;
}

static u32 desc_used(struct edev_queue *q)
{
    return ring_distance(q, q->sw_cidx, q->sw_pidx);
}

static u32 desc_free(struct edev_queue *q)
{
    return q->ring_size - 1 - desc_used(q);
}

static bool edev_queue_valid(struct edev_dev *edev, u32 qid)
{
    if (!edev || !edev->queues || qid >= edev->queue_count)
        return false;
    return READ_ONCE(edev->queues[qid].configured);
}

static bool edev_queue_is_st(const struct edev_queue *q)
{
    return q && !!(q->dir_mask & EDEV_QUEUE_TYPE_ST);
}

static bool edev_queue_is_st_d2h(const struct edev_queue *q)
{
    return edev_queue_is_st(q) && (q->dir_mask & EDEV_QUEUE_DIR_D2H) && !(q->dir_mask & EDEV_QUEUE_DIR_H2D);
}

static u32 edev_queue_desc_stride(const struct edev_queue *q)
{
    if (!edev_queue_is_st(q))
        return EDEV_MM_DESC_SIZE;
    return edev_queue_is_st_d2h(q) ? EDEV_ST_RX_DESC_SIZE : EDEV_ST_TX_DESC_SIZE;
}

static const char *edev_queue_type_name(const struct edev_queue *q)
{
    return edev_queue_is_st(q) ? "ST" : "MM";
}

static const char *edev_queue_dir_name(const struct edev_queue *q)
{
    if ((q->dir_mask & EDEV_QUEUE_DIR_BOTH) == EDEV_QUEUE_DIR_BOTH)
        return "H2D|D2H";
    return (q->dir_mask & EDEV_QUEUE_DIR_H2D) ? "H2D" : "D2H";
}

static bool edev_completion_requires_exact_len(struct edev_dev *edev, u32 qid)
{
    struct edev_queue *q;

    if (!edev_queue_valid(edev, qid))
        return true;
    q = &edev->queues[qid];
    return !edev_queue_is_st_d2h(q);
}

static int edev_set_dma_mask(struct edev_dev *edev)
{
    int ret = dma_set_mask_and_coherent(edev->dev, DMA_BIT_MASK(64));
    if (!ret) {
        dev_info(edev->dev, "using 64-bit DMA/coherent mask\n");
        return 0;
    }
    ret = dma_set_mask_and_coherent(edev->dev, DMA_BIT_MASK(32));
    if (!ret)
        dev_info(edev->dev, "using 32-bit DMA/coherent mask\n");
    return ret;
}

static int edev_get_upstream_min_mps(struct pci_dev *pdev)
{
    struct pci_bus *bus;
    int min_mps = 0;

    if (!pdev)
        return 0;

    for (bus = pdev->bus; bus; bus = bus->parent) {
        struct pci_dev *bridge = bus->self;
        int mps;

        if (!bridge)
            continue;
        mps = pcie_get_mps(bridge);
        if (mps > 0 && (!min_mps || mps < min_mps))
            min_mps = mps;
    }
    return min_mps;
}

static bool edev_upstream_has_vendor(struct pci_dev *pdev, u16 vendor)
{
    struct pci_bus *bus;

    if (!pdev)
        return false;
    for (bus = pdev->bus; bus; bus = bus->parent) {
        struct pci_dev *bridge = bus->self;
        if (bridge && bridge->vendor == vendor)
            return true;
    }
    return false;
}

static int edev_sanitize_mrrs_bytes(unsigned int requested)
{
    switch (requested) {
    case 128:
    case 256:
    case 512:
    case 1024:
    case 2048:
    case 4096:
        return (int)requested;
    default:
        return 0;
    }
}

static int edev_select_mrrs_bytes(struct pci_dev *pdev)
{
    int requested = edev_sanitize_mrrs_bytes(pcie_mrrs_bytes);

    if (requested)
        return requested;
    return edev_upstream_has_vendor(pdev, PCI_VENDOR_ID_NVIDIA) ? 256 : 512;
}

static int edev_enable_pcie_requester_caps(struct edev_dev *edev)
{
    struct pci_dev *pdev = edev->pdev;
    u32 devcap = 0;
    u16 devctl = 0;
    int ret;
    int dev_mps;
    int path_mps;
    int target_mps;
    int target_mrrs;

    ret = pcie_capability_read_dword(pdev, PCI_EXP_DEVCAP, &devcap);
    if (ret)
        return ret;
    if (!(devcap & PCI_EXP_DEVCAP_EXT_TAG)) {
        dev_err(edev->dev, "PCIe Extended Tag capability is absent\n");
        return -ENODEV;
    }

    ret = pcie_capability_read_word(pdev, PCI_EXP_DEVCTL, &devctl);
    if (ret)
        return ret;
    if (!(devctl & PCI_EXP_DEVCTL_EXT_TAG)) {
        ret = pcie_capability_set_word(pdev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_EXT_TAG);
        if (ret)
            return ret;
        ret = pcie_capability_read_word(pdev, PCI_EXP_DEVCTL, &devctl);
        if (ret)
            return ret;
    }
    if (!(devctl & PCI_EXP_DEVCTL_EXT_TAG)) {
        dev_err(edev->dev, "PCIe Extended Tag enable failed\n");
        return -EIO;
    }

    dev_mps = pcie_get_mps(pdev);
    path_mps = edev_get_upstream_min_mps(pdev);
    target_mps = dev_mps > 0 ? dev_mps : 128;
    if (path_mps > 0 && target_mps > path_mps)
        target_mps = path_mps;

    if (target_mps != dev_mps) {
        ret = pcie_set_mps(pdev, target_mps);
        if (ret) {
            dev_err(edev->dev, "failed to set path-safe MPS=%d: %d\n", target_mps, ret);
            return ret;
        }
    }

    target_mrrs = edev_select_mrrs_bytes(pdev);
    ret = pcie_set_readrq(pdev, target_mrrs);
    if (ret && target_mrrs > 256) {
        dev_warn(edev->dev, "failed MRRS=%d, retrying 256: %d\n", target_mrrs, ret);
        target_mrrs = 256;
        ret = pcie_set_readrq(pdev, target_mrrs);
    }
    if (ret)
        dev_warn(edev->dev, "failed to set MRRS=%d, keeping current value: %d\n", target_mrrs, ret);

    dev_info(edev->dev, "PCIe requester config: ExtTag enabled, MPS=%d path_min=%d MRRS=%d requested=%d\n",
             pcie_get_mps(pdev), path_mps, pcie_get_readrq(pdev), target_mrrs);
    return 0;
}

static int edev_wait_hw_idle(struct edev_dev *edev, unsigned int timeout_ms)
{
    unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
    u32 status;

    do {
        status = edev_read32(edev, EDEV_REG_G_STATUS);
        if (status == 0xffffffffu)
            return -EIO;
        if (!(status & EDEV_G_STATUS_BUSY))
            return 0;
        usleep_range(1000, 2000);
    } while (time_before(jiffies, deadline));

    status = edev_read32(edev, EDEV_REG_G_STATUS);
    if (status != 0xffffffffu && !(status & EDEV_G_STATUS_BUSY))
        return 0;
    dev_warn(edev->dev, "DMA engine did not quiesce before reset, status=0x%08x\n", status);
    return -ETIMEDOUT;
}

static int edev_wait_hw_ready(struct edev_dev *edev, unsigned int timeout_ms)
{
    unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
    u32 status;

    do {
        status = edev_read32(edev, EDEV_REG_G_STATUS);
        if (status == 0xffffffffu)
            return -EIO;
        if (status & EDEV_G_STATUS_FATAL_ERROR)
            return -EIO;
        if (status & EDEV_G_STATUS_READY)
            return 0;
        usleep_range(1000, 2000);
    } while (time_before(jiffies, deadline));

    status = edev_read32(edev, EDEV_REG_G_STATUS);
    if (status != 0xffffffffu && !(status & EDEV_G_STATUS_FATAL_ERROR) && (status & EDEV_G_STATUS_READY))
        return 0;
    dev_warn(edev->dev, "DMA engine not ready after configure, status=0x%08x\n", status);
    return -ETIMEDOUT;
}

static void edev_hw_reset_assert(struct edev_dev *edev)
{
    edev_write32(edev, EDEV_REG_G_CTRL, EDEV_CTRL_RESET);
    edev_read32(edev, EDEV_REG_G_CTRL);
    udelay(50);
    edev_write32(edev, EDEV_REG_G_CTRL, 0);
    edev_read32(edev, EDEV_REG_G_CTRL);
    udelay(50);
}

static void edev_hw_reset_force(struct edev_dev *edev)
{
    edev_write32(edev, EDEV_REG_G_IRQ_ENABLE, 0);
    edev_write32(edev, EDEV_REG_G_CTRL, 0);
    edev_write32(edev, EDEV_REG_G_QUEUE_COUNT, edev->queue_count);
    edev_read32(edev, EDEV_REG_G_CTRL);
    udelay(20);
    edev_hw_reset_assert(edev);
    edev_read32(edev, EDEV_REG_G_STATUS);
    udelay(20);
}

static int edev_map_bars(struct edev_dev *edev)
{
    int i;

    for (i = 0; i < 6; i++) {
        if (!pci_resource_start(edev->pdev, i) || !pci_resource_len(edev->pdev, i))
            continue;
        edev->bar_len[i] = pci_resource_len(edev->pdev, i);
        edev->bar[i] = pci_ioremap_bar(edev->pdev, i);
        if (!edev->bar[i])
            return -ENOMEM;
        dev_info(edev->dev, "BAR%d mapped len=%pa flags=0x%lx\n",
                 i, &edev->bar_len[i], pci_resource_flags(edev->pdev, i));
    }
    if (!edev->bar[0])
        return -ENODEV;
    if (!edev->bar[2])
        dev_warn(edev->dev, "BAR2 is not present; mmap path disabled\n");
    return 0;
}

static void edev_unmap_bars(struct edev_dev *edev)
{
    int i;

    for (i = 0; i < 6; i++) {
        if (edev->bar[i]) {
            pci_iounmap(edev->pdev, edev->bar[i]);
            edev->bar[i] = NULL;
        }
    }
}

static u32 edev_depth_field_or_one(u32 value, u32 shift)
{
    u32 field = EDEV_HW_DEPTH_FIELD(value, shift);

    if (!field)
        field = 1;
    return field;
}

static void edev_read_hw_limits(struct edev_dev *edev)
{
    u32 raw;

    edev->hw_cap = edev_read32(edev, EDEV_REG_G_CAP) | EDEV_CAP_SYNC_ONLY |
                   EDEV_CAP_SINGLE_WB_MM | EDEV_CAP_NO_REAP | EDEV_CAP_FAIR_QUEUE;
    raw = edev_read32(edev, EDEV_REG_G_HW_DEPTH);
    if (raw == 0xffffffffu || raw == 0)
        raw = EDEV_HW_DEPTH_FAIR_SYNC;

    edev->hw_depth_raw = raw;
    edev->hw_global_outstanding_limit = edev_depth_field_or_one(raw, EDEV_HW_DEPTH_GLOBAL_SHIFT);
    edev->hw_mm_outstanding_limit = min_t(u32, edev_depth_field_or_one(raw, EDEV_HW_DEPTH_MM_SHIFT), 1u);
    edev->hw_st_tx_outstanding_limit = min_t(u32, edev_depth_field_or_one(raw, EDEV_HW_DEPTH_ST_TX_SHIFT), 1u);
    edev->hw_st_rx_outstanding_limit = min_t(u32, edev_depth_field_or_one(raw, EDEV_HW_DEPTH_ST_RX_SHIFT), 1u);

    dev_info(edev->dev, "sync DMA caps: cap=0x%08x raw_depth=0x%08x global=%u mm=%u st_tx=%u st_rx=%u\n",
             edev->hw_cap, raw, edev->hw_global_outstanding_limit,
             edev->hw_mm_outstanding_limit, edev->hw_st_tx_outstanding_limit,
             edev->hw_st_rx_outstanding_limit);
}

static void edev_reset_queue_sw_state(struct edev_queue *q)
{
    mutex_lock(&q->lock);
    q->sw_pidx = 0;
    q->sw_cidx = 0;
    q->last_error = 0;
    q->busy = false;
    q->kernel_async = false;
    q->kernel_op_id = 0;
    q->kernel_target_cidx = 0;
    q->kernel_desc_count = 0;
    q->kernel_swtag = 0;
    q->kernel_recovery_epoch = 0;
    memset(&q->last_cmpl, 0, sizeof(q->last_cmpl));
    if (q->ring_mem)
        memset(q->ring_mem, 0, q->ring_mem_size);
    mutex_unlock(&q->lock);
}

static void edev_program_queue(struct edev_dev *edev, struct edev_queue *q)
{
    edev_q_write32(edev, q->qid, EDEV_QREG_CTRL, 0);
    edev_q_write32(edev, q->qid, EDEV_QREG_MODE, q->dir_mask);
    edev_q_write32(edev, q->qid, EDEV_QREG_IRQ_ENABLE,
                    edev->irq_mode ? (EDEV_IRQ_CMPL | EDEV_IRQ_ERROR) : 0);
    edev_q_write32(edev, q->qid, EDEV_QREG_DESC_BASE_LO, lower_32_bits(q->desc_dma));
    edev_q_write32(edev, q->qid, EDEV_QREG_DESC_BASE_HI, upper_32_bits(q->desc_dma));
    edev_q_write32(edev, q->qid, EDEV_QREG_CMPL_BASE_LO, lower_32_bits(q->cmpl_dma));
    edev_q_write32(edev, q->qid, EDEV_QREG_CMPL_BASE_HI, upper_32_bits(q->cmpl_dma));
    edev_q_write32(edev, q->qid, EDEV_QREG_RING_SIZE, q->ring_size);
    edev_q_write32(edev, q->qid, EDEV_QREG_DESC_PIDX, q->sw_pidx);
    edev_q_write32(edev, q->qid, EDEV_QREG_DESC_CIDX, q->sw_cidx);
    edev_q_write32(edev, q->qid, EDEV_QREG_CMPL_PIDX, q->sw_cidx);
    wmb();
    edev_q_write32(edev, q->qid, EDEV_QREG_CTRL, EDEV_CTRL_ENABLE | EDEV_CTRL_START);
    edev_q_read32(edev, q->qid, EDEV_QREG_CTRL);
    q->configured = true;
}

static void edev_hw_configure(struct edev_dev *edev)
{
    u32 mode = EDEV_MODE_SGDMA |
               ((edev->irq_mode || edev->camera_irq_mode) ? EDEV_MODE_IRQ : 0);
    u32 irq_en = (edev->irq_mode ? (EDEV_IRQ_CMPL | EDEV_IRQ_ERROR) : 0) |
                 (edev->camera_irq_mode ? EDEV_IRQ_CAMERA_FRAME : 0);
    u32 i;

    edev_write32(edev, EDEV_REG_G_MODE, mode);
    edev_write32(edev, EDEV_REG_G_IRQ_ENABLE, irq_en);
    edev_write32(edev, EDEV_REG_G_QUEUE_COUNT, edev->queue_count);
    for (i = 0; i < edev->queue_count; i++)
        edev_program_queue(edev, &edev->queues[i]);
    wmb();
    edev_write32(edev, EDEV_REG_G_CTRL, EDEV_CTRL_ENABLE | EDEV_CTRL_START);
    edev_read32(edev, EDEV_REG_G_STATUS);
    (void)edev_wait_hw_ready(edev, 100);
}

static bool edev_hw_mmio_accessible(struct edev_dev *edev)
{
    u32 status;

    if (READ_ONCE(edev->hw_offline))
        return false;
    status = edev_read32(edev, EDEV_REG_G_STATUS);
    return status != 0xffffffffu;
}

static void edev_queue_bounce_free(struct edev_queue *q);

static void edev_recover_timeout_locked(struct edev_dev *edev)
{
    u32 qid;

    if (!edev_hw_mmio_accessible(edev)) {
        wake_up_interruptible(&edev->waitq);
        return;
    }

    dev_warn(edev->dev, "recovering synchronous DMA timeout by resetting SGDMA engine\n");
    edev_hw_reset_force(edev);
    for (qid = 0; qid < edev->queue_count; qid++)
        edev_reset_queue_sw_state(&edev->queues[qid]);
    dma_wmb();
    edev_hw_configure(edev);
    atomic_inc(&edev->recovery_epoch);
    wake_up_interruptible(&edev->waitq);
}

static int edev_alloc_queues(struct edev_dev *edev)
{
    u32 i;

    edev->queues = kcalloc(edev->queue_count, sizeof(*edev->queues), GFP_KERNEL);
    if (!edev->queues)
        return -ENOMEM;

    for (i = 0; i < edev->queue_count; i++) {
        struct edev_queue *q = &edev->queues[i];

        q->edev = edev;
        q->qid = i;
        mutex_init(&q->lock);
        mutex_init(&q->bounce_lock);
        init_waitqueue_head(&q->waitq);
        if (i < st_queue_count)
            q->dir_mask = ((i < st_h2d_queue_count) ? EDEV_QUEUE_DIR_H2D : EDEV_QUEUE_DIR_D2H) | EDEV_QUEUE_TYPE_ST;
        else
            q->dir_mask = EDEV_QUEUE_DIR_BOTH | EDEV_QUEUE_TYPE_MM;
        q->ring_size = ring_size;
        q->ring_mask = ring_size - 1;
        q->ring_mem_size = q->ring_size * edev_queue_desc_stride(q) + sizeof(*q->cmpl);
        q->ring_mem = dma_alloc_coherent(edev->dev, q->ring_mem_size, &q->ring_dma, GFP_KERNEL);
        if (!q->ring_mem)
            return -ENOMEM;
        q->desc = (struct edev_hw_desc *)q->ring_mem;
        q->desc_dma = q->ring_dma;
        q->cmpl = (struct edev_hw_wb *)((u8 *)q->ring_mem + q->ring_size * edev_queue_desc_stride(q));
        q->cmpl_dma = q->ring_dma + q->ring_size * edev_queue_desc_stride(q);
        memset(q->ring_mem, 0, q->ring_mem_size);
        dev_info(edev->dev, "queue %u: type=%s dir=%s desc_stride=%u desc=%pad wb=%pad ring=%u block=%zu\n",
                 i, edev_queue_type_name(q), edev_queue_dir_name(q), edev_queue_desc_stride(q),
                 &q->desc_dma, &q->cmpl_dma, q->ring_size, q->ring_mem_size);
    }
    return 0;
}

static void edev_free_queues(struct edev_dev *edev)
{
    u32 i;

    if (!edev->queues)
        return;
    for (i = 0; i < edev->queue_count; i++) {
        struct edev_queue *q = &edev->queues[i];
        edev_queue_bounce_free(q);
        if (q->ring_mem)
            dma_free_coherent(edev->dev, q->ring_mem_size, q->ring_mem, q->ring_dma);
    }
    kfree(edev->queues);
    edev->queues = NULL;
}

static void edev_unpin_pages(struct page **pages, int npages, bool dirty)
{
    int i;

    for (i = 0; i < npages; i++) {
        if (!pages[i])
            continue;
        if (dirty)
            set_page_dirty_lock(pages[i]);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
        unpin_user_page(pages[i]);
#else
        put_page(pages[i]);
#endif
    }
}

static void edev_release_user_buffer(struct edev_dev *edev, struct edev_user_buffer *buf)
{
    if (!buf)
        return;

    if (buf->imported_dmabuf) {
        if (buf->dmabuf_attach && buf->dmabuf_sgt)
            dma_buf_unmap_attachment(buf->dmabuf_attach, buf->dmabuf_sgt, buf->dma_dir);
        if (buf->dmabuf && buf->dmabuf_attach)
            dma_buf_detach(buf->dmabuf, buf->dmabuf_attach);
        if (buf->dmabuf)
            dma_buf_put(buf->dmabuf);
        kfree(buf);
        return;
    }

    if (buf->nmap)
        dma_unmap_sg(edev->dev, buf->sgt.sgl, buf->sgt.nents, buf->dma_dir);
    sg_free_table(&buf->sgt);
    if (buf->pages)
        edev_unpin_pages(buf->pages, buf->npages, !buf->h2d);
    kvfree(buf->pages);
    kfree(buf);
}


static size_t edev_effective_bounce_chunk(void)
{
    size_t chunk = bounce_chunk_bytes ? bounce_chunk_bytes : EDEV_DEFAULT_BOUNCE_BYTES;

    if (chunk < PAGE_SIZE)
        chunk = PAGE_SIZE;
    return chunk;
}

static void edev_queue_bounce_free(struct edev_queue *q)
{
    if (!q)
        return;
    if (q->bounce_sgt_valid) {
        sg_free_table(&q->bounce_sgt);
        q->bounce_sgt_valid = false;
    }
    if (q->bounce_cpu) {
        dma_free_coherent(q->edev->dev, q->bounce_size, q->bounce_cpu, q->bounce_dma);
        q->bounce_cpu = NULL;
        q->bounce_dma = 0;
        q->bounce_size = 0;
    }
}

static int edev_queue_bounce_ensure(struct edev_queue *q, size_t size)
{
    struct scatterlist *sg;
    size_t alloc_size;
    int ret;

    if (!q || !size)
        return -EINVAL;
    alloc_size = PAGE_ALIGN(size);
    if (q->bounce_cpu && q->bounce_size >= alloc_size)
        return 0;

    edev_queue_bounce_free(q);

    q->bounce_cpu = dma_alloc_coherent(q->edev->dev, alloc_size, &q->bounce_dma, GFP_KERNEL);
    if (!q->bounce_cpu)
        return -ENOMEM;
    q->bounce_size = alloc_size;

    ret = sg_alloc_table(&q->bounce_sgt, 1, GFP_KERNEL);
    if (ret) {
        edev_queue_bounce_free(q);
        return ret;
    }

    sg = q->bounce_sgt.sgl;
    sg->offset = 0;
    sg->length = q->bounce_size;
    sg_dma_address(sg) = q->bounce_dma;
    sg_dma_len(sg) = q->bounce_size;
    q->bounce_sgt_valid = true;
    return 0;
}

static void edev_sync_user_buffer_range(struct edev_dev *edev,
                                        struct edev_user_buffer *buf,
                                        u64 buf_offset, size_t count,
                                        bool for_device)
{
    struct scatterlist *sg;
    u64 skip = buf_offset;
    size_t remaining = count;
    int i;

    if (!edev || !buf || !remaining)
        return;

    for_each_sg(buf->sgt.sgl, sg, buf->nmap, i) {
        dma_addr_t dma = sg_dma_address(sg);
        size_t len = sg_dma_len(sg);
        size_t off;
        size_t n;

        if (skip >= len) {
            skip -= len;
            continue;
        }
        off = (size_t)skip;
        skip = 0;
        n = min_t(size_t, len - off, remaining);

        if (for_device)
            dma_sync_single_range_for_device(edev->dev, dma, off, n, buf->dma_dir);
        else
            dma_sync_single_range_for_cpu(edev->dev, dma, off, n, buf->dma_dir);

        remaining -= n;
        if (!remaining)
            break;
    }
}

static struct edev_user_buffer *edev_find_buffer(struct edev_file_ctx *ctx, u32 id)
{
    struct edev_user_buffer *buf;

    list_for_each_entry(buf, &ctx->buffers, node) {
        if (buf->id == id)
            return buf;
    }
    return NULL;
}

static int edev_register_user_buffer(struct edev_file_ctx *ctx, struct edev_ioc_user_buffer __user *uarg)
{
    struct edev_ioc_user_buffer arg;
    struct edev_user_buffer *buf;
    unsigned long start, first, last;
    unsigned int gup_flags;
    int ret;

    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EFAULT;
    if (!arg.user_va || !arg.size || (max_registered_bytes && arg.size > max_registered_bytes))
        return -EINVAL;
    if (arg.dir != EDEV_DMA_DIR_H2D && arg.dir != EDEV_DMA_DIR_D2H)
        return -EINVAL;

    mutex_lock(&ctx->buf_lock);
    if (max_registered_buffers && ctx->buffer_count >= max_registered_buffers) {
        mutex_unlock(&ctx->buf_lock);
        return -ENOMEM;
    }
    mutex_unlock(&ctx->buf_lock);

    buf = kzalloc(sizeof(*buf), GFP_KERNEL);
    if (!buf)
        return -ENOMEM;
    buf->h2d = arg.dir == EDEV_DMA_DIR_H2D;
    buf->user_va = (unsigned long)arg.user_va;
    buf->size = arg.size;
    buf->dma_dir = buf->h2d ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

    start = buf->user_va;
    first = start & PAGE_MASK;
    last = PAGE_ALIGN(start + buf->size);
    if (last <= first) {
        ret = -EINVAL;
        goto err_free;
    }
    buf->npages = (last - first) >> PAGE_SHIFT;
    buf->pages = kvcalloc(buf->npages, sizeof(struct page *), GFP_KERNEL);
    if (!buf->pages) {
        ret = -ENOMEM;
        goto err_free;
    }

    gup_flags = buf->h2d ? 0 : FOLL_WRITE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
    ret = pin_user_pages_fast(first, buf->npages, gup_flags, buf->pages);
#else
    ret = get_user_pages_fast(first, buf->npages, gup_flags, buf->pages);
#endif
    if (ret < 0)
        goto err_free;
    if (ret != buf->npages) {
        buf->npages = ret;
        ret = -EFAULT;
        goto err_unpin;
    }

    ret = sg_alloc_table_from_pages(&buf->sgt, buf->pages, buf->npages,
                                    offset_in_page(start), buf->size, GFP_KERNEL);
    if (ret)
        goto err_unpin;
    buf->nmap = dma_map_sg(ctx->edev->dev, buf->sgt.sgl, buf->sgt.nents, buf->dma_dir);
    if (!buf->nmap) {
        ret = -EIO;
        goto err_sg;
    }

    dma_sync_sg_for_cpu(ctx->edev->dev, buf->sgt.sgl, buf->sgt.nents, buf->dma_dir);

    mutex_lock(&ctx->buf_lock);
    buf->id = ++ctx->next_buf_id;
    list_add_tail(&buf->node, &ctx->buffers);
    ctx->buffer_count++;
    mutex_unlock(&ctx->buf_lock);

    arg.buf_id = buf->id;
    arg.nmap = buf->nmap;
    if (copy_to_user(uarg, &arg, sizeof(arg))) {
        mutex_lock(&ctx->buf_lock);
        list_del(&buf->node);
        ctx->buffer_count--;
        mutex_unlock(&ctx->buf_lock);
        edev_release_user_buffer(ctx->edev, buf);
        return -EFAULT;
    }
    return 0;

err_sg:
    sg_free_table(&buf->sgt);
err_unpin:
    edev_unpin_pages(buf->pages, buf->npages, false);
err_free:
    kvfree(buf->pages);
    kfree(buf);
    return ret;
}

static int edev_register_dmabuf(struct edev_file_ctx *ctx, struct edev_ioc_dmabuf __user *uarg)
{
    struct edev_ioc_dmabuf arg;
    struct edev_user_buffer *buf;
    enum dma_data_direction dir;
    int ret;

    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EFAULT;
    if (arg.fd < 0 || !arg.size)
        return -EINVAL;
    if (arg.dir != EDEV_DMA_DIR_H2D && arg.dir != EDEV_DMA_DIR_D2H)
        return -EINVAL;
    dir = (arg.dir == EDEV_DMA_DIR_H2D) ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

    mutex_lock(&ctx->buf_lock);
    if (max_registered_buffers && ctx->buffer_count >= max_registered_buffers) {
        mutex_unlock(&ctx->buf_lock);
        return -ENOMEM;
    }
    mutex_unlock(&ctx->buf_lock);

    buf = kzalloc(sizeof(*buf), GFP_KERNEL);
    if (!buf)
        return -ENOMEM;
    buf->imported_dmabuf = true;
    buf->h2d = arg.dir == EDEV_DMA_DIR_H2D;
    buf->dma_dir = dir;
    buf->user_va = 0;
    buf->size = arg.size;

    buf->dmabuf = dma_buf_get(arg.fd);
    if (IS_ERR(buf->dmabuf)) {
        ret = PTR_ERR(buf->dmabuf);
        buf->dmabuf = NULL;
        goto err_free;
    }
    if (arg.offset != 0 || arg.offset > (u64)buf->dmabuf->size ||
        arg.size > (u64)buf->dmabuf->size - arg.offset) {
        ret = -EINVAL;
        goto err_release;
    }

    /* This driver imports NvBufSurface DMA-BUFs for device-to-device transfers
     * only: FPGA PCIe SGDMA writes the buffer and GPU/EGL consumes it.  Do not
     * bracket registration with dma_buf_begin/end_cpu_access(); those calls are
     * for CPU mappings and can trigger expensive or unsupported cache-maintenance
     * paths in NvMap exporters.  The attachment map below is the proper DMA
     * importer operation for the SGDMA device.
     */
    buf->dmabuf_attach = dma_buf_attach(buf->dmabuf, ctx->edev->dev);
    if (IS_ERR(buf->dmabuf_attach)) {
        ret = PTR_ERR(buf->dmabuf_attach);
        buf->dmabuf_attach = NULL;
        goto err_release;
    }
    buf->dmabuf_sgt = dma_buf_map_attachment(buf->dmabuf_attach, dir);
    if (IS_ERR(buf->dmabuf_sgt)) {
        ret = PTR_ERR(buf->dmabuf_sgt);
        buf->dmabuf_sgt = NULL;
        goto err_release;
    }

    buf->sgt = *buf->dmabuf_sgt;
    buf->nmap = buf->sgt.nents;

    mutex_lock(&ctx->buf_lock);
    buf->id = ++ctx->next_buf_id;
    list_add_tail(&buf->node, &ctx->buffers);
    ctx->buffer_count++;
    mutex_unlock(&ctx->buf_lock);

    arg.buf_id = buf->id;
    arg.nmap = buf->nmap;
    if (copy_to_user(uarg, &arg, sizeof(arg))) {
        mutex_lock(&ctx->buf_lock);
        list_del(&buf->node);
        ctx->buffer_count--;
        mutex_unlock(&ctx->buf_lock);
        edev_release_user_buffer(ctx->edev, buf);
        return -EFAULT;
    }
    return 0;

err_release:
    edev_release_user_buffer(ctx->edev, buf);
    return ret;
err_free:
    kfree(buf);
    return ret;
}

static int edev_unregister_user_buffer(struct edev_file_ctx *ctx, u32 __user *uarg)
{
    u32 id;
    struct edev_user_buffer *buf, *tmp;

    if (copy_from_user(&id, uarg, sizeof(id)))
        return -EFAULT;
    mutex_lock(&ctx->buf_lock);
    list_for_each_entry_safe(buf, tmp, &ctx->buffers, node) {
        if (buf->id == id) {
            if (buf->active) {
                mutex_unlock(&ctx->buf_lock);
                return -EBUSY;
            }
            list_del(&buf->node);
            ctx->buffer_count--;
            mutex_unlock(&ctx->buf_lock);
            edev_release_user_buffer(ctx->edev, buf);
            return 0;
        }
    }
    mutex_unlock(&ctx->buf_lock);
    return -ENOENT;
}

static int edev_pin_user_buffer(struct edev_file_ctx *ctx, unsigned long user_va,
                                size_t size, u32 dir, struct edev_user_buffer **out)
{
    struct edev_user_buffer *buf;
    unsigned long first, last;
    unsigned int gup_flags;
    int ret;

    if (!user_va || !size || (max_registered_bytes && size > max_registered_bytes))
        return -EINVAL;
    if (dir != EDEV_DMA_DIR_H2D && dir != EDEV_DMA_DIR_D2H)
        return -EINVAL;

    buf = kzalloc(sizeof(*buf), GFP_KERNEL);
    if (!buf)
        return -ENOMEM;
    buf->h2d = dir == EDEV_DMA_DIR_H2D;
    buf->user_va = user_va;
    buf->size = size;
    buf->dma_dir = buf->h2d ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

    first = user_va & PAGE_MASK;
    last = PAGE_ALIGN(user_va + size);
    if (last <= first) {
        ret = -EINVAL;
        goto err_free;
    }
    buf->npages = (last - first) >> PAGE_SHIFT;
    buf->pages = kvcalloc(buf->npages, sizeof(struct page *), GFP_KERNEL);
    if (!buf->pages) {
        ret = -ENOMEM;
        goto err_free;
    }

    gup_flags = buf->h2d ? 0 : FOLL_WRITE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
    ret = pin_user_pages_fast(first, buf->npages, gup_flags, buf->pages);
#else
    ret = get_user_pages_fast(first, buf->npages, gup_flags, buf->pages);
#endif
    if (ret < 0)
        goto err_free;
    if (ret != buf->npages) {
        buf->npages = ret;
        ret = -EFAULT;
        goto err_unpin;
    }

    ret = sg_alloc_table_from_pages(&buf->sgt, buf->pages, buf->npages,
                                    offset_in_page(user_va), size, GFP_KERNEL);
    if (ret)
        goto err_unpin;
    buf->nmap = dma_map_sg(ctx->edev->dev, buf->sgt.sgl, buf->sgt.nents, buf->dma_dir);
    if (!buf->nmap) {
        ret = -EIO;
        goto err_sg;
    }
    *out = buf;
    return 0;

err_sg:
    sg_free_table(&buf->sgt);
err_unpin:
    edev_unpin_pages(buf->pages, buf->npages, false);
err_free:
    kvfree(buf->pages);
    kfree(buf);
    return ret;
}

static u32 edev_effective_fair_desc_bytes(const struct edev_queue *q)
{
    unsigned long limit = fair_desc_bytes ? fair_desc_bytes : EDEV_DEFAULT_FAIR_DESC_BYTES;

    if (limit < PAGE_SIZE)
        limit = PAGE_SIZE;
    if (limit > EDEV_MAX_DESC_LEN)
        limit = EDEV_MAX_DESC_LEN;
    if (edev_queue_is_st(q) && limit > U32_MAX)
        limit = U32_MAX;
    return (u32)limit;
}

static int edev_fill_mm_descs_append(struct edev_queue *q,
                                      u32 *pidx_io, u32 *desc_no_io,
                                      struct sg_table *sgt, int nmap,
                                      bool h2d, u64 dev_addr, u64 buf_offset,
                                      size_t count, u32 max_descs, u16 swtag,
                                      bool sop, bool eop, bool irq_on_done,
                                      u32 total_bytes32, u32 *entry_desc_count)
{
    struct scatterlist *sg;
    u64 skip = buf_offset;
    u64 cur_dev = dev_addr;
    size_t remaining = count;
    u32 fair_chunk_bytes = edev_effective_fair_desc_bytes(q);
    u32 entry_descs = 0;
    u32 pidx;
    u32 desc_no;
    int i;

    if (!q || edev_queue_is_st(q) || !pidx_io || !desc_no_io || !entry_desc_count)
        return -EINVAL;
    if (!count || count > U32_MAX || !max_descs)
        return -EMSGSIZE;

    pidx = *pidx_io;
    desc_no = *desc_no_io;

    for_each_sg(sgt->sgl, sg, nmap, i) {
        dma_addr_t dma = sg_dma_address(sg);
        u32 len = sg_dma_len(sg);
        u32 consumed = 0;

        if (skip >= len) {
            skip -= len;
            continue;
        }
        dma += skip;
        len -= skip;
        skip = 0;

        while (consumed < len && remaining) {
            struct edev_hw_desc *d;
            u32 chunk = min_t(u32, len - consumed, fair_chunk_bytes);
            u16 desc_flags = EDEV_DESC_VALID | EDEV_DESC_ERROR_STOP;

            chunk = min_t(u32, chunk, remaining);
            if (desc_no >= max_descs)
                return -EMSGSIZE;

            if (!h2d)
                desc_flags |= EDEV_DESC_DIR_D2H;
            if (sop && entry_descs == 0)
                desc_flags |= EDEV_DESC_SOP;
            if (eop && chunk == remaining) {
                desc_flags |= EDEV_DESC_EOP;
                if (irq_on_done)
                    desc_flags |= EDEV_DESC_IRQ_ON_DONE;
            }

            d = &q->desc[pidx & q->ring_mask];
            memset(d, 0, sizeof(*d));
            d->host_addr = cpu_to_le64(dma + consumed);
            d->dev_addr = cpu_to_le64(cur_dev);
            d->len = cpu_to_le32(chunk);
            d->tag = cpu_to_le16(swtag);
            d->flags = cpu_to_le16(desc_flags);
            d->seq = cpu_to_le32(desc_no);
            d->rsvd = cpu_to_le32(total_bytes32);

            pidx = ring_next(q, pidx);
            cur_dev += chunk;
            consumed += chunk;
            remaining -= chunk;
            desc_no++;
            entry_descs++;
        }
        if (!remaining)
            break;
    }

    if (remaining || !entry_descs)
        return -EINVAL;

    *pidx_io = pidx;
    *desc_no_io = desc_no;
    *entry_desc_count = entry_descs;
    return 0;
}

static int edev_fill_descs(struct edev_queue *q, u32 start_pidx, struct sg_table *sgt, int nmap,
                           bool h2d, u64 dev_addr, u64 buf_offset, size_t count,
                           u32 max_descs, u16 swtag, bool irq_on_done,
                           u32 *desc_count, u32 *target_cidx)
{
    struct scatterlist *sg;
    u32 total_bytes32;
    u32 fair_chunk_bytes = edev_effective_fair_desc_bytes(q);
    u32 desc_no = 0;
    u32 pidx = start_pidx;
    int i;
    int ret;

    if (!count || !max_descs)
        return -EMSGSIZE;
    if (count > U32_MAX)
        return -EMSGSIZE;
    total_bytes32 = (u32)count;

    if (edev_queue_is_st(q)) {
        for_each_sg(sgt->sgl, sg, nmap, i) {
            dma_addr_t dma = sg_dma_address(sg);
            u32 sg_len = sg_dma_len(sg);
            u32 slot = pidx & q->ring_mask;

            if (buf_offset >= sg_len) {
                buf_offset -= sg_len;
                continue;
            }
            if (count > sg_len - buf_offset || max_descs < 1)
                return -EMSGSIZE;
            if (count > fair_chunk_bytes)
                return -EMSGSIZE;

            if (h2d) {
                struct edev_st_tx_desc *d = (struct edev_st_tx_desc *)((u8 *)q->desc + slot * EDEV_ST_TX_DESC_SIZE);
                memset(d, 0, sizeof(*d));
                d->host_addr = cpu_to_le64(dma + buf_offset);
                d->len = cpu_to_le32((u32)count);
                d->tag = cpu_to_le16(swtag);
                d->qid = cpu_to_le16((u16)q->qid);
            } else {
                struct edev_st_rx_desc *d = (struct edev_st_rx_desc *)((u8 *)q->desc + slot * EDEV_ST_RX_DESC_SIZE);
                memset(d, 0, sizeof(*d));
                d->host_addr = cpu_to_le64(dma + buf_offset);
                d->len = cpu_to_le32((u32)count);
                d->tag = cpu_to_le16(swtag);
                d->qid = cpu_to_le16((u16)q->qid);
            }

            *desc_count = 1;
            *target_cidx = ring_next(q, pidx);
            return 0;
        }
        return -EINVAL;
    }

    ret = edev_fill_mm_descs_append(q, &pidx, &desc_no, sgt, nmap, h2d,
                                    dev_addr, buf_offset, count, max_descs,
                                    swtag, true, true, irq_on_done,
                                    total_bytes32, desc_count);
    if (ret)
        return ret;

    *target_cidx = pidx;
    return 0;
}

static bool edev_wb_valid(struct edev_queue *q)
{
    return !!(le32_to_cpu(READ_ONCE(q->cmpl->status)) & EDEV_WB_STATUS_VALID);
}

static int edev_wait_queue_wb(struct edev_dev *edev, struct edev_queue *q)
{
    unsigned int poll_us = sync_poll_us ? sync_poll_us : EDEV_SYNC_POLL_US_DEFAULT;
    unsigned long deadline = 0;

    if (sync_timeout_ms)
        deadline = jiffies + msecs_to_jiffies(sync_timeout_ms);

    /* For 1 MiB-class synchronous DMA operations the fixed completion wait
     * latency dominates measured bandwidth.  A short busy-poll window keeps the
     * submit thread on CPU until the queue WB appears, then falls back to the
     * old sleep/wait path for long or stalled operations.
     */
    if (!edev->irq_mode && sync_busy_poll_us) {
        u64 spin_deadline = ktime_get_ns() + (u64)sync_busy_poll_us * 1000ull;

        do {
            if (edev_wb_valid(q))
                return 0;
            cpu_relax();
        } while (ktime_get_ns() < spin_deadline);
    }

    for (;;) {
        if (edev_wb_valid(q))
            return 0;
        if (sync_timeout_ms && time_after_eq(jiffies, deadline)) {
            dev_err(edev->dev, "queue %u: synchronous DMA timed out waiting for WB\n", q->qid);
            return -ETIMEDOUT;
        }
        if (edev->irq_mode) {
            (void)wait_event_timeout(q->waitq, edev_wb_valid(q), msecs_to_jiffies(1));
        } else {
            usleep_range(poll_us, poll_us + 10);
        }
    }
}

static void edev_fill_submit_completion(struct edev_queue *q, struct edev_cmpl_cache *cmpl,
                                        u64 op_id, u32 desc_count, u16 swtag,
                                        u32 flags, u32 completed_cidx, u32 submitted_cidx,
                                        u32 bytes_done, u32 error_code, int status)
{
    memset(cmpl, 0, sizeof(*cmpl));
    cmpl->op_id = op_id;
    cmpl->bytes_done = status ? 0 : bytes_done;
    cmpl->status = status;
    cmpl->flags = flags;
    cmpl->error_code = error_code;
    cmpl->desc_cidx = completed_cidx;
    cmpl->submitted_cidx = submitted_cidx;
    cmpl->desc_count = desc_count;
    cmpl->swtag = swtag;
    q->last_cmpl = *cmpl;
}

static int edev_consume_wb(struct edev_queue *q, u64 op_id, u32 target_cidx,
                           u32 desc_count, u16 expected_swtag,
                           struct edev_cmpl_cache *cmpl)
{
    struct edev_hw_wb *wb = q->cmpl;
    u32 status;
    u32 hw_qid;
    u32 swtag;
    u32 completed_cidx;
    u32 submitted_cidx;
    u32 bytes_done;
    u32 error_code;
    bool bad_wb = false;
    int ret;

    dma_rmb();
    status = le32_to_cpu(READ_ONCE(wb->status));
    hw_qid = le32_to_cpu(READ_ONCE(wb->qid));
    swtag = le32_to_cpu(READ_ONCE(wb->swtag));
    completed_cidx = le32_to_cpu(READ_ONCE(wb->completed_cidx)) & q->ring_mask;
    submitted_cidx = le32_to_cpu(READ_ONCE(wb->submitted_cidx)) & q->ring_mask;
    bytes_done = le32_to_cpu(READ_ONCE(wb->bytes));
    error_code = le32_to_cpu(READ_ONCE(wb->error_code));

    if (!(status & EDEV_WB_STATUS_VALID)) {
        bad_wb = true;
        error_code = 0xdead0200u;
    } else if ((hw_qid & (EDEV_MAX_QUEUES - 1)) != q->qid) {
        bad_wb = true;
        error_code = 0xdead0201u;
    } else if (((u16)swtag) != expected_swtag) {
        bad_wb = true;
        error_code = 0xdead0202u;
    } else if (!(status & EDEV_WB_STATUS_ERROR) && completed_cidx != (target_cidx & q->ring_mask)) {
        bad_wb = true;
        error_code = 0xdead0203u;
    }

    if (bad_wb) {
        status = EDEV_WB_STATUS_VALID | EDEV_WB_STATUS_ERROR | EDEV_WB_STATUS_SOP | EDEV_WB_STATUS_EOP;
        if (edev_queue_is_st(q))
            status |= EDEV_WB_STATUS_TARGET_ST;
        completed_cidx = target_cidx & q->ring_mask;
        submitted_cidx = target_cidx & q->ring_mask;
        bytes_done = 0;
        ret = -EIO;
    } else {
        ret = (status & EDEV_WB_STATUS_ERROR) ? -EIO : 0;
    }

    q->sw_cidx = completed_cidx;
    q->last_error = ret ? error_code : 0;
    WRITE_ONCE(wb->status, 0);
    dma_wmb();

    edev_fill_submit_completion(q, cmpl, op_id, desc_count, (u16)swtag, status,
                                completed_cidx, submitted_cidx, bytes_done,
                                error_code, ret);
    return ret;
}

u32 edev_kernel_st_tx_count(struct edev_dev *edev)
{
    return edev ? min_t(u32, st_h2d_queue_count, edev->queue_count) : 0;
}

u32 edev_kernel_st_rx_base(struct edev_dev *edev)
{
    return edev ? min_t(u32, st_h2d_queue_count, edev->queue_count) : 0;
}

u32 edev_kernel_st_rx_count(struct edev_dev *edev)
{
    u32 end;

    if (!edev)
        return 0;
    end = min_t(u32, st_queue_count, edev->queue_count);
    return end > st_h2d_queue_count ? end - st_h2d_queue_count : 0;
}

int edev_kernel_async_submit(struct edev_dev *edev, u32 qid,
                             dma_addr_t host_addr, u32 len, bool h2d)
{
    struct edev_queue *q;
    u32 slot;
    u32 target_cidx;
    u64 op_id;
    u16 swtag;
    int ret = 0;

    if (!edev_queue_valid(edev, qid) || !host_addr || !len)
        return -EINVAL;
    q = &edev->queues[qid];
    if (!edev_queue_is_st(q) ||
        (h2d && !(q->dir_mask & EDEV_QUEUE_DIR_H2D)) ||
        (!h2d && !(q->dir_mask & EDEV_QUEUE_DIR_D2H)))
        return -EINVAL;
    if (len > edev_effective_fair_desc_bytes(q))
        return -EMSGSIZE;
    if (READ_ONCE(edev->hw_offline))
        return -ENODEV;

    down_read(&edev->submit_rwsem);
    mutex_lock(&q->lock);
    if (q->busy) {
        ret = -EBUSY;
        goto out;
    }
    if (!desc_free(q)) {
        ret = -ENOSPC;
        goto out;
    }

    q->busy = true;
    q->kernel_async = true;
    memset(q->cmpl, 0, sizeof(*q->cmpl));
    op_id = atomic64_inc_return(&edev->next_op_id);
    swtag = (u16)op_id;
    slot = q->sw_pidx & q->ring_mask;
    target_cidx = ring_next(q, q->sw_pidx);
    if (h2d) {
        struct edev_st_tx_desc *d =
            (struct edev_st_tx_desc *)((u8 *)q->desc +
                                       slot * EDEV_ST_TX_DESC_SIZE);
        memset(d, 0, sizeof(*d));
        d->host_addr = cpu_to_le64(host_addr);
        d->len = cpu_to_le32(len);
        d->tag = cpu_to_le16(swtag);
        d->qid = cpu_to_le16((u16)qid);
    } else {
        struct edev_st_rx_desc *d =
            (struct edev_st_rx_desc *)((u8 *)q->desc +
                                       slot * EDEV_ST_RX_DESC_SIZE);
        memset(d, 0, sizeof(*d));
        d->host_addr = cpu_to_le64(host_addr);
        d->len = cpu_to_le32(len);
        d->tag = cpu_to_le16(swtag);
        d->qid = cpu_to_le16((u16)qid);
    }

    q->kernel_op_id = op_id;
    q->kernel_target_cidx = target_cidx;
    q->kernel_desc_count = 1;
    q->kernel_swtag = swtag;
    q->kernel_recovery_epoch = atomic_read(&edev->recovery_epoch);
    q->sw_pidx = target_cidx;
    dma_wmb();
    edev_q_write32(edev, qid, EDEV_QREG_DESC_PIDX, target_cidx);

out:
    mutex_unlock(&q->lock);
    up_read(&edev->submit_rwsem);
    return ret;
}

int edev_kernel_async_poll(struct edev_dev *edev, u32 qid,
                           struct edev_kernel_cmpl *cmpl)
{
    struct edev_queue *q;
    struct edev_cmpl_cache cache;
    int ret;

    if (!edev_queue_valid(edev, qid) || !cmpl)
        return -EINVAL;
    q = &edev->queues[qid];
    down_read(&edev->submit_rwsem);
    mutex_lock(&q->lock);
    if (!q->busy || !q->kernel_async) {
        ret = -ENOENT;
        goto out;
    }
    if (q->kernel_recovery_epoch != atomic_read(&edev->recovery_epoch)) {
        q->busy = false;
        q->kernel_async = false;
        ret = -EIO;
        goto out;
    }
    if (!edev_wb_valid(q)) {
        ret = 0;
        goto out;
    }

    cmpl->user = le32_to_cpu(READ_ONCE(q->cmpl->user));
    ret = edev_consume_wb(q, q->kernel_op_id, q->kernel_target_cidx,
                          q->kernel_desc_count, q->kernel_swtag, &cache);
    cmpl->status = cache.flags;
    cmpl->error_code = cache.error_code;
    cmpl->bytes = (u32)cache.bytes_done;
    q->busy = false;
    q->kernel_async = false;
    ret = ret ? ret : 1;
out:
    mutex_unlock(&q->lock);
    up_read(&edev->submit_rwsem);
    return ret;
}

void edev_kernel_async_cancel(struct edev_dev *edev, u32 qid)
{
    struct edev_queue *q;

    if (!edev || qid >= edev->queue_count)
        return;
    q = &edev->queues[qid];
    mutex_lock(&q->lock);
    if (q->kernel_async) {
        q->busy = false;
        q->kernel_async = false;
        memset(q->cmpl, 0, sizeof(*q->cmpl));
    }
    mutex_unlock(&q->lock);
}

static void edev_user_buffer_active_dec(struct edev_file_ctx *ctx, struct edev_user_buffer *buf)
{
    mutex_lock(&ctx->buf_lock);
    if (buf->active)
        buf->active--;
    mutex_unlock(&ctx->buf_lock);
}

static int edev_submit_buffer_sync(struct edev_file_ctx *ctx, struct edev_user_buffer *buf,
                                   u32 qid, u32 dir, u32 flags_in, u64 buf_offset,
                                   u64 len, u64 dev_addr, u64 *op_id_out,
                                   u32 *desc_start_out, u32 *desc_count_out,
                                   bool active_preincremented,
                                   struct edev_cmpl_cache *cmpl)
{
    struct edev_dev *edev = ctx->edev;
    struct edev_queue *q;
    bool h2d;
    bool irq_on_done;
    bool skip_cpu_sync;
    u32 total_descs = 0;
    u32 start_pidx = 0;
    u32 target_cidx = 0;
    u32 max_descs = 0;
    u64 op_id;
    u16 swtag;
    int submit_epoch = 0;
    int ret = 0;

    if (!buf)
        return -EINVAL;
    if (!active_preincremented) {
        mutex_lock(&ctx->buf_lock);
        buf->active++;
        mutex_unlock(&ctx->buf_lock);
    }

    if (!edev_queue_valid(edev, qid) || !len) {
        ret = -EINVAL;
        goto out_deactive;
    }
    if (dir != EDEV_DMA_DIR_H2D && dir != EDEV_DMA_DIR_D2H) {
        ret = -EINVAL;
        goto out_deactive;
    }
    if (flags_in & ~EDEV_SUBMIT_F_VALID_MASK) {
        ret = -EINVAL;
        goto out_deactive;
    }

    q = &edev->queues[qid];
    h2d = dir == EDEV_DMA_DIR_H2D;
    irq_on_done = edev->irq_mode && !(flags_in & EDEV_SUBMIT_F_NO_IRQ);
    skip_cpu_sync = !!(flags_in & EDEV_SUBMIT_F_SKIP_CPU_SYNC);
    if (h2d && !(q->dir_mask & EDEV_QUEUE_DIR_H2D)) {
        ret = -EINVAL;
        goto out_deactive;
    }
    if (!h2d && !(q->dir_mask & EDEV_QUEUE_DIR_D2H)) {
        ret = -EINVAL;
        goto out_deactive;
    }
    if (buf->h2d != h2d || buf_offset > buf->size || len > buf->size - buf_offset) {
        ret = -EINVAL;
        goto out_deactive;
    }
    if (READ_ONCE(edev->hw_offline)) {
        ret = -ENODEV;
        goto out_deactive;
    }
    if (edev_queue_is_st(q) && edev->netdev) {
        ret = -EBUSY;
        goto out_deactive;
    }

    submit_epoch = atomic_read(&edev->recovery_epoch);
    down_read(&edev->submit_rwsem);
    mutex_lock(&q->lock);
    if (q->busy) {
        mutex_unlock(&q->lock);
        up_read(&edev->submit_rwsem);
        ret = -EBUSY;
        goto out_deactive;
    }
    q->busy = true;
    memset(q->cmpl, 0, sizeof(*q->cmpl));
    dma_wmb();

    op_id = atomic64_inc_return(&edev->next_op_id);
    swtag = (u16)op_id;
    start_pidx = q->sw_pidx;
    max_descs = desc_free(q);
    if (!max_descs) {
        ret = -ENOSPC;
        goto out_unlock_q;
    }

    if (!skip_cpu_sync)
        edev_sync_user_buffer_range(edev, buf, buf_offset, len, true);
    ret = edev_fill_descs(q, start_pidx, &buf->sgt, buf->nmap, h2d,
                          dev_addr, buf_offset, len, max_descs, swtag,
                          irq_on_done, &total_descs, &target_cidx);
    if (ret)
        goto out_sync_cpu;
    if (total_descs >= q->ring_size) {
        ret = -EMSGSIZE;
        goto out_sync_cpu;
    }

    q->sw_pidx = target_cidx;
    dma_wmb();
    edev_q_write32(edev, q->qid, EDEV_QREG_DESC_PIDX, target_cidx);

    ret = edev_wait_queue_wb(edev, q);
    if (ret == -ETIMEDOUT) {
        q->last_error = 0xdead0300u;
        if (!skip_cpu_sync && !h2d)
            edev_sync_user_buffer_range(edev, buf, buf_offset, len, false);
        if (op_id_out)
            *op_id_out = op_id;
        if (desc_start_out)
            *desc_start_out = start_pidx;
        if (desc_count_out)
            *desc_count_out = total_descs;
        q->busy = false;
        wake_up_interruptible(&q->waitq);
        up_read(&edev->submit_rwsem);
        mutex_unlock(&q->lock);
        down_write(&edev->submit_rwsem);
        mutex_lock(&edev->sched_lock);
        if (atomic_read(&edev->recovery_epoch) == submit_epoch && !READ_ONCE(edev->hw_offline)) {
            edev_recover_timeout_locked(edev);
        } else {
            dev_warn(edev->dev, "queue %u: timeout recovery already handled by another submitter\n", q->qid);
        }
        mutex_unlock(&edev->sched_lock);
        up_write(&edev->submit_rwsem);
        edev_user_buffer_active_dec(ctx, buf);
        return ret;
    }
    if (!ret)
        ret = edev_consume_wb(q, op_id, target_cidx, total_descs, swtag, cmpl);

    /* H2D buffers are not modified by the device, so returning ownership with
     * dma_sync_*_for_cpu() after every completed H2D operation only adds cache
     * maintenance overhead to throughput tests.  D2H still requires the CPU-side
     * sync so software can safely consume the received data.
     */
    if (!skip_cpu_sync && !h2d)
        edev_sync_user_buffer_range(edev, buf, buf_offset, len, false);

    if (op_id_out)
        *op_id_out = op_id;
    if (desc_start_out)
        *desc_start_out = start_pidx;
    if (desc_count_out)
        *desc_count_out = total_descs;

    q->busy = false;
    wake_up_interruptible(&q->waitq);
    up_read(&edev->submit_rwsem);
    mutex_unlock(&q->lock);
    edev_user_buffer_active_dec(ctx, buf);
    return ret;

out_sync_cpu:
    if (!skip_cpu_sync && !h2d)
        edev_sync_user_buffer_range(edev, buf, buf_offset, len, false);
out_unlock_q:
    q->busy = false;
    wake_up_interruptible(&q->waitq);
    up_read(&edev->submit_rwsem);
    mutex_unlock(&q->lock);
out_deactive:
    edev_user_buffer_active_dec(ctx, buf);
    return ret;
}


static int edev_fill_mm_descs_2d(struct edev_queue *q, u32 start_pidx,
                                 struct sg_table *sgt, int nmap, bool h2d,
                                 u64 dev_addr, u64 buf_offset,
                                 u32 row_bytes, u32 rows,
                                 u32 dev_stride, u32 buf_stride,
                                 u32 max_descs, u16 swtag, bool irq_on_done,
                                 u32 *desc_count, u32 *target_cidx)
{
    u32 pidx = start_pidx;
    u32 desc_no = 0;
    u32 total_bytes32;
    u32 y;
    u64 total_bytes;
    int ret;

    if (!q || !sgt || !desc_count || !target_cidx || edev_queue_is_st(q))
        return -EINVAL;
    if (!row_bytes || !rows || !dev_stride || !buf_stride || !max_descs)
        return -EINVAL;
    if (dev_stride < row_bytes || buf_stride < row_bytes)
        return -EINVAL;

    total_bytes = (u64)row_bytes * (u64)rows;
    if (total_bytes > U32_MAX)
        return -EMSGSIZE;
    total_bytes32 = (u32)total_bytes;

    for (y = 0; y < rows; y++) {
        u32 entry_descs = 0;
        u64 row_dev = dev_addr + (u64)y * (u64)dev_stride;
        u64 row_buf = buf_offset + (u64)y * (u64)buf_stride;
        bool first = (y == 0);
        bool last = (y == rows - 1u);

        if (row_dev + row_bytes < row_dev || row_buf + row_bytes < row_buf)
            return -EINVAL;

        ret = edev_fill_mm_descs_append(q, &pidx, &desc_no, sgt, nmap, h2d,
                                        row_dev, row_buf, row_bytes,
                                        max_descs, swtag, first, last,
                                        last && irq_on_done,
                                        total_bytes32, &entry_descs);
        if (ret)
            return ret;
    }

    if (!desc_no)
        return -EINVAL;
    *desc_count = desc_no;
    *target_cidx = pidx;
    return 0;
}

static int edev_submit_buffer_2d_sync(struct edev_file_ctx *ctx,
                                      struct edev_user_buffer *buf,
                                      struct edev_ioc_dma_submit_2d *arg,
                                      bool active_preincremented,
                                      struct edev_cmpl_cache *cmpl)
{
    struct edev_dev *edev = ctx->edev;
    struct edev_queue *q;
    bool h2d;
    bool irq_on_done;
    bool skip_cpu_sync;
    u32 total_descs = 0;
    u32 start_pidx = 0;
    u32 target_cidx = 0;
    u32 max_descs = 0;
    u64 total_bytes;
    u64 buf_extent;
    u64 dev_extent;
    u64 op_id;
    u16 swtag;
    int submit_epoch = 0;
    int ret = 0;

    if (!buf || !arg)
        return -EINVAL;
    if (!active_preincremented) {
        mutex_lock(&ctx->buf_lock);
        buf->active++;
        mutex_unlock(&ctx->buf_lock);
    }

    if (!edev_queue_valid(edev, arg->qid) || !arg->row_bytes || !arg->rows) {
        ret = -EINVAL;
        goto out_deactive;
    }
    if (arg->dir != EDEV_DMA_DIR_D2H) {
        /* The retained Orin RGB8 path only needs FPGA-DDR -> NvBufSurface. */
        ret = -EINVAL;
        goto out_deactive;
    }
    if (arg->flags & ~EDEV_SUBMIT_F_VALID_MASK) {
        ret = -EINVAL;
        goto out_deactive;
    }
    if (!arg->dev_stride)
        arg->dev_stride = arg->row_bytes;
    if (!arg->buf_stride)
        arg->buf_stride = arg->row_bytes;
    if (arg->dev_stride < arg->row_bytes || arg->buf_stride < arg->row_bytes) {
        ret = -EINVAL;
        goto out_deactive;
    }

    total_bytes = (u64)arg->row_bytes * (u64)arg->rows;
    if (!total_bytes || total_bytes > U32_MAX) {
        ret = -EMSGSIZE;
        goto out_deactive;
    }
    buf_extent = (u64)(arg->rows - 1u) * (u64)arg->buf_stride + arg->row_bytes;
    dev_extent = (u64)(arg->rows - 1u) * (u64)arg->dev_stride + arg->row_bytes;
    if (buf_extent < arg->row_bytes || dev_extent < arg->row_bytes ||
        arg->buf_offset > buf->size || buf_extent > buf->size - arg->buf_offset) {
        ret = -EINVAL;
        goto out_deactive;
    }

    q = &edev->queues[arg->qid];
    if (edev_queue_is_st(q)) {
        ret = -EINVAL;
        goto out_deactive;
    }
    h2d = false;
    irq_on_done = edev->irq_mode && !(arg->flags & EDEV_SUBMIT_F_NO_IRQ);
    skip_cpu_sync = !!(arg->flags & EDEV_SUBMIT_F_SKIP_CPU_SYNC);
    if (!(q->dir_mask & EDEV_QUEUE_DIR_D2H)) {
        ret = -EINVAL;
        goto out_deactive;
    }
    if (buf->h2d || READ_ONCE(edev->hw_offline)) {
        ret = buf->h2d ? -EINVAL : -ENODEV;
        goto out_deactive;
    }

    submit_epoch = atomic_read(&edev->recovery_epoch);
    down_read(&edev->submit_rwsem);
    mutex_lock(&q->lock);
    q->busy = true;
    memset(q->cmpl, 0, sizeof(*q->cmpl));
    dma_wmb();

    op_id = atomic64_inc_return(&edev->next_op_id);
    swtag = (u16)op_id;
    start_pidx = q->sw_pidx;
    max_descs = desc_free(q);
    if (!max_descs) {
        ret = -ENOSPC;
        goto out_unlock_q;
    }

    if (!skip_cpu_sync)
        edev_sync_user_buffer_range(edev, buf, arg->buf_offset,
                                    (size_t)buf_extent, true);
    ret = edev_fill_mm_descs_2d(q, start_pidx, &buf->sgt, buf->nmap, h2d,
                                arg->dev_addr, arg->buf_offset,
                                arg->row_bytes, arg->rows,
                                arg->dev_stride, arg->buf_stride,
                                max_descs, swtag, irq_on_done,
                                &total_descs, &target_cidx);
    if (ret)
        goto out_sync_cpu;
    if (total_descs >= q->ring_size) {
        ret = -EMSGSIZE;
        goto out_sync_cpu;
    }

    q->sw_pidx = target_cidx;
    dma_wmb();
    edev_q_write32(edev, q->qid, EDEV_QREG_DESC_PIDX, target_cidx);

    ret = edev_wait_queue_wb(edev, q);
    if (ret == -ETIMEDOUT) {
        q->last_error = 0xdead0400u;
        if (!skip_cpu_sync)
            edev_sync_user_buffer_range(edev, buf, arg->buf_offset,
                                        (size_t)buf_extent, false);
        arg->op_id = op_id;
        arg->desc_start = start_pidx;
        arg->desc_count = total_descs;
        arg->bytes_done = 0;
        q->busy = false;
        wake_up_interruptible(&q->waitq);
        up_read(&edev->submit_rwsem);
        mutex_unlock(&q->lock);
        down_write(&edev->submit_rwsem);
        mutex_lock(&edev->sched_lock);
        if (atomic_read(&edev->recovery_epoch) == submit_epoch && !READ_ONCE(edev->hw_offline))
            edev_recover_timeout_locked(edev);
        else
            dev_warn(edev->dev, "queue %u: 2D timeout recovery already handled by another submitter\n", q->qid);
        mutex_unlock(&edev->sched_lock);
        up_write(&edev->submit_rwsem);
        edev_user_buffer_active_dec(ctx, buf);
        return ret;
    }
    if (!ret)
        ret = edev_consume_wb(q, op_id, target_cidx, total_descs, swtag, cmpl);

    if (!skip_cpu_sync)
        edev_sync_user_buffer_range(edev, buf, arg->buf_offset,
                                    (size_t)buf_extent, false);

    arg->op_id = op_id;
    arg->desc_start = start_pidx;
    arg->desc_count = total_descs;
    arg->bytes_done = cmpl ? cmpl->bytes_done : 0;

    q->busy = false;
    wake_up_interruptible(&q->waitq);
    up_read(&edev->submit_rwsem);
    mutex_unlock(&q->lock);
    edev_user_buffer_active_dec(ctx, buf);
    return ret;

out_sync_cpu:
    if (!skip_cpu_sync)
        edev_sync_user_buffer_range(edev, buf, arg->buf_offset,
                                    (size_t)buf_extent, false);
out_unlock_q:
    q->busy = false;
    wake_up_interruptible(&q->waitq);
    up_read(&edev->submit_rwsem);
    mutex_unlock(&q->lock);
out_deactive:
    edev_user_buffer_active_dec(ctx, buf);
    return ret;
}

static int edev_submit_2d(struct edev_file_ctx *ctx,
                          struct edev_ioc_dma_submit_2d __user *uarg)
{
    struct edev_ioc_dma_submit_2d arg;
    struct edev_user_buffer *buf;
    struct edev_cmpl_cache cmpl;
    int ret;

    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EFAULT;

    mutex_lock(&ctx->buf_lock);
    buf = edev_find_buffer(ctx, arg.buf_id);
    if (buf)
        buf->active++;
    mutex_unlock(&ctx->buf_lock);
    if (!buf) {
        arg.status = -ENOENT;
        if (copy_to_user(uarg, &arg, sizeof(arg)))
            return -EFAULT;
        return -ENOENT;
    }

    memset(&cmpl, 0, sizeof(cmpl));
    ret = edev_submit_buffer_2d_sync(ctx, buf, &arg, true, &cmpl);
    if (!ret && cmpl.bytes_done != (u32)((u64)arg.row_bytes * (u64)arg.rows))
        ret = -EIO;
    arg.status = ret;
    arg.bytes_done = cmpl.bytes_done;
    if (copy_to_user(uarg, &arg, sizeof(arg)))
        return -EFAULT;
    return ret;
}

static int edev_submit_registered(struct edev_file_ctx *ctx, struct edev_ioc_dma_submit __user *uarg)
{
    struct edev_ioc_dma_submit arg;
    struct edev_user_buffer *buf;
    struct edev_cmpl_cache cmpl;
    int ret;

    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EFAULT;

    mutex_lock(&ctx->buf_lock);
    buf = edev_find_buffer(ctx, arg.buf_id);
    if (buf)
        buf->active++;
    mutex_unlock(&ctx->buf_lock);
    if (!buf) {
        arg.status = -ENOENT;
        if (copy_to_user(uarg, &arg, sizeof(arg)))
            return -EFAULT;
        return -ENOENT;
    }

    ret = edev_submit_buffer_sync(ctx, buf, arg.qid, arg.dir, arg.flags, arg.buf_offset,
                                  arg.len, arg.dev_addr, &arg.op_id,
                                  &arg.desc_start, &arg.desc_count, true, &cmpl);
    if (!ret && cmpl.bytes_done != arg.len &&
        edev_completion_requires_exact_len(ctx->edev, arg.qid))
        ret = -EIO;
    arg.status = ret;
    arg.reserved = 0;
    if (copy_to_user(uarg, &arg, sizeof(arg)))
        return -EFAULT;
    return ret;
}


static int edev_submit_bounce(struct edev_file_ctx *ctx, struct edev_ioc_bounce_submit __user *uarg)
{
    struct edev_ioc_bounce_submit arg;
    struct edev_dev *edev = ctx->edev;
    struct edev_queue *q;
    struct edev_user_buffer bounce;
    struct edev_cmpl_cache cmpl;
    u64 done = 0;
    u64 dma_ns = 0;
    u64 copy_ns = 0;
    u32 desc_total = 0;
    u64 last_op_id = 0;
    size_t chunk_limit;
    bool h2d;
    int ret = 0;

    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EFAULT;
    if (!arg.user_va || !arg.len || arg.user_va + arg.len < arg.user_va ||
        arg.dev_addr + arg.len < arg.dev_addr) {
        arg.status = -EINVAL;
        goto out_copy;
    }
    if (arg.dir != EDEV_DMA_DIR_H2D && arg.dir != EDEV_DMA_DIR_D2H) {
        arg.status = -EINVAL;
        goto out_copy;
    }
    if (arg.flags & ~(u32)EDEV_SUBMIT_F_NO_IRQ) {
        arg.status = -EINVAL;
        goto out_copy;
    }
    if (!edev_queue_valid(edev, arg.qid)) {
        arg.status = -EINVAL;
        goto out_copy;
    }
    if (dev_mem_size && (arg.dev_addr > dev_mem_size || arg.len > dev_mem_size - arg.dev_addr)) {
        arg.status = -EINVAL;
        goto out_copy;
    }
    if (READ_ONCE(edev->hw_offline)) {
        arg.status = -ENODEV;
        goto out_copy;
    }

    q = &edev->queues[arg.qid];
    h2d = arg.dir == EDEV_DMA_DIR_H2D;
    if (h2d && !(q->dir_mask & EDEV_QUEUE_DIR_H2D)) {
        arg.status = -EINVAL;
        goto out_copy;
    }
    if (!h2d && !(q->dir_mask & EDEV_QUEUE_DIR_D2H)) {
        arg.status = -EINVAL;
        goto out_copy;
    }

    chunk_limit = edev_effective_bounce_chunk();
    if (!chunk_limit) {
        arg.status = -EINVAL;
        goto out_copy;
    }

    mutex_lock(&q->bounce_lock);
    ret = edev_queue_bounce_ensure(q, min_t(u64, arg.len, chunk_limit));
    if (ret)
        goto out_unlock;

    while (done < arg.len) {
        size_t chunk = (size_t)min_t(u64, arg.len - done, q->bounce_size);
        u64 t0, t1;
        u32 desc_count = 0;

        memset(&bounce, 0, sizeof(bounce));
        bounce.h2d = h2d;
        bounce.size = chunk;
        bounce.nmap = 1;
        bounce.sgt = q->bounce_sgt;
        bounce.dma_dir = DMA_BIDIRECTIONAL;

        if (h2d) {
            t0 = ktime_get_ns();
            if (copy_from_user(q->bounce_cpu, (void __user *)(unsigned long)(arg.user_va + done), chunk)) {
                ret = -EFAULT;
                break;
            }
            t1 = ktime_get_ns();
            copy_ns += t1 - t0;
        }

        t0 = ktime_get_ns();
        ret = edev_submit_buffer_sync(ctx, &bounce, arg.qid, arg.dir,
                                      (arg.flags & EDEV_SUBMIT_F_NO_IRQ) | EDEV_SUBMIT_F_SKIP_CPU_SYNC,
                                      0, chunk, arg.dev_addr + done,
                                      &last_op_id, NULL, &desc_count, true, &cmpl);
        t1 = ktime_get_ns();
        dma_ns += t1 - t0;
        desc_total += desc_count;
        if (ret)
            break;
        if (edev_completion_requires_exact_len(edev, arg.qid) && cmpl.bytes_done != chunk) {
            ret = -EIO;
            break;
        }

        if (!h2d) {
            t0 = ktime_get_ns();
            if (copy_to_user((void __user *)(unsigned long)(arg.user_va + done), q->bounce_cpu, chunk)) {
                ret = -EFAULT;
                break;
            }
            t1 = ktime_get_ns();
            copy_ns += t1 - t0;
        }

        done += chunk;
    }

out_unlock:
    mutex_unlock(&q->bounce_lock);
    arg.op_id = last_op_id;
    arg.bytes_done = done;
    arg.dma_ns = dma_ns;
    arg.copy_ns = copy_ns;
    arg.desc_count = desc_total;
    arg.status = ret;
out_copy:
    if (copy_to_user(uarg, &arg, sizeof(arg)))
        return -EFAULT;
    return arg.status;
}


/* SUBMIT_BATCH is kept as a sequential compatibility wrapper only.
 * The fused hardware-op batch path was removed because performance testing now
 * measures one synchronous DMA per queue per round, and long fused D2H windows
 * can hide posted-write drain latency behind a single completion WB.
 */

static int edev_submit_batch(struct edev_file_ctx *ctx, struct edev_ioc_submit_batch __user *uarg)
{
    struct edev_ioc_submit_batch batch;
    struct edev_ioc_dma_submit __user *entries;
    u32 i, limit;
    int ret = 0;

    if (copy_from_user(&batch, uarg, sizeof(batch)))
        return -EFAULT;

    limit = ioctl_batch_limit ? ioctl_batch_limit : EDEV_BATCH_MAX;
    if (limit > EDEV_BATCH_MAX)
        limit = EDEV_BATCH_MAX;

    if (!batch.entries || !batch.count || batch.count > limit || (batch.flags & ~EDEV_BATCH_F_IRQ_LAST)) {
        batch.status = -EINVAL;
        batch.submitted = 0;
        if (copy_to_user(uarg, &batch, sizeof(batch)))
            return -EFAULT;
        return -EINVAL;
    }

    entries = (struct edev_ioc_dma_submit __user *)(unsigned long)batch.entries;

    /* Keep SUBMIT_BATCH as a compatibility ioctl only.  The fused hardware-op
     * path is intentionally disabled: performance testing now submits one
     * synchronous DMA per measured round, and long fused D2H windows can hide
     * posted-write drain latency behind a single WB.
     */

    batch.submitted = 0;
    for (i = 0; i < batch.count; i++) {
        struct edev_ioc_dma_submit arg;
        struct edev_user_buffer *buf;
        struct edev_cmpl_cache cmpl;

        if (copy_from_user(&arg, &entries[i], sizeof(arg))) {
            ret = -EFAULT;
            break;
        }

        mutex_lock(&ctx->buf_lock);
        buf = edev_find_buffer(ctx, arg.buf_id);
        if (buf)
            buf->active++;
        mutex_unlock(&ctx->buf_lock);

        if (!buf) {
            arg.status = -ENOENT;
            ret = -ENOENT;
        } else {
            ret = edev_submit_buffer_sync(ctx, buf, arg.qid, arg.dir, arg.flags,
                                          arg.buf_offset, arg.len, arg.dev_addr,
                                          &arg.op_id, &arg.desc_start,
                                          &arg.desc_count, true, &cmpl);
            if (!ret && cmpl.bytes_done != arg.len &&
                edev_completion_requires_exact_len(ctx->edev, arg.qid))
                ret = -EIO;
            arg.status = ret;
        }
        arg.reserved = 0;
        if (copy_to_user(&entries[i], &arg, sizeof(arg))) {
            ret = -EFAULT;
            break;
        }
        if (ret)
            break;
        batch.submitted = i + 1;
    }

    batch.status = ret;
    if (copy_to_user(uarg, &batch, sizeof(batch)))
        return -EFAULT;
    return ret;
}

static int edev_reap_cached(struct edev_file_ctx *ctx,
                            struct edev_ioc_dma_cmpl __user *uarg)
{
    struct edev_ioc_dma_cmpl arg;
    struct edev_queue *q;
    struct edev_cmpl_cache cache;

    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EFAULT;
    if (!edev_queue_valid(ctx->edev, arg.qid))
        return -EINVAL;
    q = &ctx->edev->queues[arg.qid];
    mutex_lock(&q->lock);
    cache = q->last_cmpl;
    mutex_unlock(&q->lock);
    if (!cache.op_id || (arg.op_id && arg.op_id != cache.op_id))
        return -EAGAIN;
    arg.op_id = cache.op_id;
    arg.bytes_done = cache.bytes_done;
    arg.status = cache.status;
    arg.flags = cache.flags;
    arg.error_code = cache.error_code;
    arg.desc_cidx = cache.desc_cidx;
    arg.submitted_cidx = cache.submitted_cidx;
    arg.desc_count = cache.desc_count;
    arg.swtag = cache.swtag;
    return copy_to_user(uarg, &arg, sizeof(arg)) ? -EFAULT : 0;
}

static int edev_reap_cached_batch(struct edev_file_ctx *ctx,
                                  struct edev_ioc_reap_batch __user *uarg)
{
    struct edev_ioc_reap_batch batch;
    struct edev_ioc_dma_cmpl __user *entries;
    u32 i;
    int ret = 0;

    if (copy_from_user(&batch, uarg, sizeof(batch)))
        return -EFAULT;
    if (!batch.entries || !batch.count || batch.count > EDEV_BATCH_MAX || batch.flags)
        return -EINVAL;
    entries = (struct edev_ioc_dma_cmpl __user *)(unsigned long)batch.entries;
    batch.completed = 0;
    for (i = 0; i < batch.count; i++) {
        ret = edev_reap_cached(ctx, &entries[i]);
        if (ret)
            break;
        batch.completed++;
    }
    batch.status = ret;
    if (copy_to_user(uarg, &batch, sizeof(batch)))
        return -EFAULT;
    return ret;
}

static void edev_free_coherent(struct edev_file_ctx *ctx)
{
    if (!ctx || !ctx->coh_cpu)
        return;
    dma_free_coherent(ctx->edev->dev, ctx->coh_size,
                      ctx->coh_cpu, ctx->coh_dma);
    ctx->coh_cpu = NULL;
    ctx->coh_dma = 0;
    ctx->coh_size = 0;
}

static int edev_alloc_coherent_compat(struct edev_file_ctx *ctx,
                                      struct reconic_dma_alloc_req __user *uarg)
{
    struct reconic_dma_alloc_req req;
    size_t size;

    if (copy_from_user(&req, uarg, sizeof(req)))
        return -EFAULT;
    if (!req.size || req.size > SIZE_MAX - (PAGE_SIZE - 1))
        return -EINVAL;
    size = PAGE_ALIGN((size_t)req.size);
    if (max_registered_bytes && size > max_registered_bytes)
        return -EMSGSIZE;
    edev_free_coherent(ctx);
    ctx->coh_cpu = dma_alloc_coherent(ctx->edev->dev, size,
                                      &ctx->coh_dma, GFP_KERNEL);
    if (!ctx->coh_cpu)
        return -ENOMEM;
    ctx->coh_size = size;
    req.size = size;
    req.dma_addr = ctx->coh_dma;
    req.mmap_offset = 0;
    if (copy_to_user(uarg, &req, sizeof(req))) {
        edev_free_coherent(ctx);
        return -EFAULT;
    }
    return 0;
}

static int edev_bar2_compat(struct edev_file_ctx *ctx,
                            struct reconic_bar2_access __user *uarg,
                            bool write)
{
    struct reconic_bar2_access access;
    int ret;

    if (copy_from_user(&access, uarg, sizeof(access)))
        return -EFAULT;
    ret = write ? edev_bar2_write32(ctx->edev, access.offset, access.value) :
                  edev_bar2_read32(ctx->edev, access.offset, &access.value);
    if (ret)
        return ret;
    if (!write && copy_to_user(uarg, &access, sizeof(access)))
        return -EFAULT;
    return 0;
}

static u32 edev_busy_queue_count(struct edev_dev *edev)
{
    u32 qid;
    u32 count = 0;

    for (qid = 0; qid < edev->queue_count; qid++) {
        struct edev_queue *q = &edev->queues[qid];
        bool busy;

        mutex_lock(&q->lock);
        busy = q->busy;
        mutex_unlock(&q->lock);
        if (busy)
            count++;
    }
    return count;
}


static int edev_queue_config(struct edev_dev *edev, struct edev_ioc_queue_cfg __user *uarg)
{
    struct edev_ioc_queue_cfg arg;
    struct edev_queue *q;

    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EFAULT;
    if (arg.qid >= edev->queue_count)
        return -EINVAL;
    if (!(arg.dir_mask & EDEV_QUEUE_DIR_BOTH) || (arg.dir_mask & ~EDEV_QUEUE_DIR_BOTH))
        return -EINVAL;
    if (arg.flags & ~EDEV_QUEUE_TYPE_MASK)
        return -EINVAL;
    if (arg.ring_size && arg.ring_size != edev->queues[arg.qid].ring_size)
        return -EINVAL;

    q = &edev->queues[arg.qid];
    if ((arg.flags & EDEV_QUEUE_TYPE_MASK) && ((arg.flags & EDEV_QUEUE_TYPE_MASK) != (q->dir_mask & EDEV_QUEUE_TYPE_MASK)))
        return -EINVAL;
    if (edev_queue_is_st(q) && arg.dir_mask == EDEV_QUEUE_DIR_BOTH)
        return -EINVAL;
    if (edev_queue_is_st(q) && ((arg.dir_mask & EDEV_QUEUE_DIR_BOTH) != (q->dir_mask & EDEV_QUEUE_DIR_BOTH)))
        return -EINVAL;

    mutex_lock(&q->lock);
    if (q->busy) {
        mutex_unlock(&q->lock);
        return -EBUSY;
    }
    if (!edev_hw_mmio_accessible(edev)) {
        mutex_unlock(&q->lock);
        return -ENODEV;
    }
    q->dir_mask = (arg.dir_mask & EDEV_QUEUE_DIR_BOTH) | (q->dir_mask & EDEV_QUEUE_TYPE_MASK);
    edev_program_queue(edev, q);
    mutex_unlock(&q->lock);
    (void)edev_wait_hw_ready(edev, 100);
    return 0;
}

static int edev_queue_stats(struct edev_dev *edev, struct edev_ioc_queue_stats __user *uarg)
{
    struct edev_ioc_queue_stats arg;
    struct edev_queue *q;

    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EFAULT;
    if (!edev_queue_valid(edev, arg.qid))
        return -EINVAL;
    if (READ_ONCE(edev->hw_offline))
        return -ENODEV;
    q = &edev->queues[arg.qid];

    memset(&arg, 0, sizeof(arg));
    arg.qid = q->qid;
    arg.status = edev_q_read32(edev, q->qid, EDEV_QREG_STATUS);
    arg.error_code = edev_q_read32(edev, q->qid, EDEV_QREG_ERROR_CODE);
    mutex_lock(&q->lock);
    arg.desc_pidx = q->sw_pidx;
    arg.desc_cidx = q->sw_cidx;
    arg.cmpl_pidx = edev_q_read32(edev, q->qid, EDEV_QREG_CMPL_PIDX);
    arg.cmpl_cidx = q->sw_cidx;
    arg.sw_desc_used = desc_used(q);
    arg.sw_cmpl_pending = q->busy ? 1u : 0u;
    arg.sw_cmpl_done = 0;
    mutex_unlock(&q->lock);
    arg.perf_desc_fetch = edev_q_read32(edev, q->qid, EDEV_QREG_PERF_DESC_FETCH);
    arg.perf_dma_submit = edev_q_read32(edev, q->qid, EDEV_QREG_PERF_DMA_SUBMIT);
    arg.perf_wb_write = edev_q_read32(edev, q->qid, EDEV_QREG_PERF_WB_WRITE);
    arg.perf_bytes = edev_q_read32(edev, q->qid, EDEV_QREG_PERF_BYTES);
    return copy_to_user(uarg, &arg, sizeof(arg)) ? -EFAULT : 0;
}

static int edev_get_caps(struct edev_dev *edev, struct edev_ioc_hw_caps __user *uarg)
{
    struct edev_ioc_hw_caps arg;
    u32 batch_limit;

    memset(&arg, 0, sizeof(arg));
    batch_limit = ioctl_batch_limit ? ioctl_batch_limit : EDEV_BATCH_MAX;
    if (batch_limit > EDEV_BATCH_MAX)
        batch_limit = EDEV_BATCH_MAX;

    arg.cap = edev->hw_cap | EDEV_CAP_SYNC_ONLY | EDEV_CAP_SINGLE_WB_MM |
              EDEV_CAP_NO_REAP | EDEV_CAP_FAIR_QUEUE;
    arg.queue_count = edev->queue_count;
    arg.ring_size = ring_size;
    arg.hw_depth_raw = edev->hw_depth_raw ? edev->hw_depth_raw : EDEV_HW_DEPTH_FAIR_SYNC;
    arg.hw_global_outstanding = edev->queue_count > 255 ? 255 : edev->queue_count;
    arg.hw_mm_outstanding = 1;
    arg.hw_st_tx_outstanding = 1;
    arg.hw_st_rx_outstanding = 1;
    arg.sw_global_outstanding = arg.hw_global_outstanding;
    arg.sw_global_live = edev_busy_queue_count(edev);
    arg.mm_queue_limit = 1;
    arg.st_queue_limit = 1;
    arg.ioctl_batch_limit = batch_limit;
    return copy_to_user(uarg, &arg, sizeof(arg)) ? -EFAULT : 0;
}

static inline u32 edev_camera_read32(struct edev_dev *edev, u64 csr_base, u32 reg)
{
    if (!edev || !edev->bar[2] || csr_base + reg + 4 > edev->bar_len[2])
        return 0xffffffffu;
    return ioread32(edev->bar[2] + csr_base + reg);
}

static inline void edev_camera_write32(struct edev_dev *edev, u64 csr_base, u32 reg, u32 val)
{
    if (!edev || !edev->bar[2] || csr_base + reg + 4 > edev->bar_len[2])
        return;
    iowrite32(val, edev->bar[2] + csr_base + reg);
}

static bool edev_camera_ready(struct edev_dev *edev, u64 csr_base,
                              struct edev_ioc_camera_wait *w)
{
    u32 prod = edev_camera_read32(edev, csr_base, EDEV_CAMERA_REG_PRODUCER_IDX);
    u32 cons = edev_camera_read32(edev, csr_base, EDEV_CAMERA_REG_CONSUMER_IDX);
    u32 status = edev_camera_read32(edev, csr_base, EDEV_CAMERA_REG_STATUS);

    if (w) {
        w->producer_idx = prod;
        w->consumer_idx = cons;
        w->status = status;
        w->width = edev_camera_read32(edev, csr_base, EDEV_CAMERA_REG_WIDTH);
        w->height = edev_camera_read32(edev, csr_base, EDEV_CAMERA_REG_HEIGHT);
        w->frame_bytes = edev_camera_read32(edev, csr_base, EDEV_CAMERA_REG_FRAME_BYTES);
        w->drop_count = edev_camera_read32(edev, csr_base, EDEV_CAMERA_REG_DROP_COUNT);
    }
    if (prod == 0xffffffffu || cons == 0xffffffffu || status == 0xffffffffu)
        return false;
    if (status & EDEV_CAMERA_STATUS_ERROR_MASK)
        return true;
    return prod != cons;
}

static int edev_camera_wait_frame(struct edev_file_ctx *ctx,
                                  struct edev_ioc_camera_wait __user *uarg)
{
    struct edev_ioc_camera_wait w;
    u64 base;
    long ret;

    if (copy_from_user(&w, uarg, sizeof(w)))
        return -EFAULT;
    base = w.csr_base ? w.csr_base : camera_csr_base;
    if (!ctx || !ctx->edev || !ctx->edev->bar[2] || base >= ctx->edev->bar_len[2])
        return -ENODEV;

    edev_camera_write32(ctx->edev, base, EDEV_CAMERA_REG_IRQ_ENABLE, EDEV_CAMERA_IRQ_FRAME_READY);

    if (!edev_camera_ready(ctx->edev, base, &w)) {
        unsigned long tout = w.timeout_ms ? msecs_to_jiffies(w.timeout_ms) : MAX_SCHEDULE_TIMEOUT;
        ret = wait_event_interruptible_timeout(ctx->edev->camera_waitq,
                                               edev_camera_ready(ctx->edev, base, NULL), tout);
        if (ret < 0) {
            w.result = ret;
            if (copy_to_user(uarg, &w, sizeof(w)))
                return -EFAULT;
            return ret;
        }
        if (ret == 0) {
            (void)edev_camera_ready(ctx->edev, base, &w);
            w.result = -ETIMEDOUT;
            if (copy_to_user(uarg, &w, sizeof(w)))
                return -EFAULT;
            return -ETIMEDOUT;
        }
    }

    (void)edev_camera_ready(ctx->edev, base, &w);
    w.result = (w.status & EDEV_CAMERA_STATUS_ERROR_MASK) ? -EPIPE : 0;
    if (copy_to_user(uarg, &w, sizeof(w)))
        return -EFAULT;
    return w.result;
}

static int edev_camera_submit_frame(struct edev_file_ctx *ctx,
                                    struct edev_ioc_camera_submit_frame __user *uarg)
{
    struct edev_ioc_camera_submit_frame arg;
    struct edev_user_buffer *buf;
    struct edev_cmpl_cache cmpl;
    struct edev_dev *edev;
    u64 base;
    u32 prod, cons, status;
    u32 ring_slots, stride, slot;
    u64 ring_base;
    unsigned long tout;
    int ret = 0;

    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EFAULT;
    if (!ctx || !ctx->edev) {
        ret = -ENODEV;
        goto out_copy;
    }
    edev = ctx->edev;
    base = arg.csr_base ? arg.csr_base : camera_csr_base;
    if (!edev->bar[2] || base >= edev->bar_len[2]) {
        ret = -ENODEV;
        goto out_copy;
    }
    if (!arg.len || arg.qid >= edev->queue_count) {
        ret = -EINVAL;
        goto out_copy;
    }

    mutex_lock(&ctx->buf_lock);
    buf = edev_find_buffer(ctx, arg.buf_id);
    if (buf)
        buf->active++;
    mutex_unlock(&ctx->buf_lock);
    if (!buf) {
        ret = -ENOENT;
        goto out_copy;
    }
    if (buf->h2d || arg.buf_offset > buf->size || arg.len > buf->size - arg.buf_offset) {
        ret = -EINVAL;
        goto out_deactive;
    }

    edev_camera_write32(edev, base, EDEV_CAMERA_REG_IRQ_ENABLE, EDEV_CAMERA_IRQ_FRAME_READY);

    mutex_lock(&edev->camera_lock);
    for (;;) {
        prod = edev_camera_read32(edev, base, EDEV_CAMERA_REG_PRODUCER_IDX);
        cons = edev_camera_read32(edev, base, EDEV_CAMERA_REG_CONSUMER_IDX);
        status = edev_camera_read32(edev, base, EDEV_CAMERA_REG_STATUS);
        if (prod == 0xffffffffu || cons == 0xffffffffu || status == 0xffffffffu) {
            ret = -EIO;
            break;
        }
        if (status & EDEV_CAMERA_STATUS_ERROR_MASK) {
            arg.producer_idx = prod;
            arg.consumer_idx = cons;
            arg.status = status;
            arg.width = edev_camera_read32(edev, base, EDEV_CAMERA_REG_WIDTH);
            arg.height = edev_camera_read32(edev, base, EDEV_CAMERA_REG_HEIGHT);
            arg.frame_bytes = edev_camera_read32(edev, base, EDEV_CAMERA_REG_FRAME_BYTES);
            arg.drop_count = edev_camera_read32(edev, base, EDEV_CAMERA_REG_DROP_COUNT);
            ret = -EPIPE;
            break;
        }
        if (prod != cons)
            break;
        tout = arg.timeout_ms ? msecs_to_jiffies(arg.timeout_ms) : MAX_SCHEDULE_TIMEOUT;
        mutex_unlock(&edev->camera_lock);
        ret = wait_event_interruptible_timeout(edev->camera_waitq,
                                               edev_camera_ready(edev, base, NULL),
                                               tout);
        mutex_lock(&edev->camera_lock);
        if (ret < 0)
            break;
        if (ret == 0) {
            ret = -ETIMEDOUT;
            break;
        }
    }
    if (ret < 0) {
        mutex_unlock(&edev->camera_lock);
        goto out_deactive;
    }

    ring_base = arg.ring_base ? arg.ring_base : edev_camera_read32(edev, base, EDEV_CAMERA_REG_RING_BASE_LO);
    stride = arg.frame_stride ? arg.frame_stride : edev_camera_read32(edev, base, EDEV_CAMERA_REG_FRAME_STRIDE);
    ring_slots = arg.ring_slots ? arg.ring_slots : edev_camera_read32(edev, base, EDEV_CAMERA_REG_RING_SLOTS);
    if (!ring_slots)
        ring_slots = 32; /* matches reconic_camera_capture CAMERA_RING_SLOTS */
    if (!stride) {
        ret = -EINVAL;
        mutex_unlock(&edev->camera_lock);
        goto out_deactive;
    }
    slot = cons % ring_slots;
    arg.producer_idx = prod;
    arg.consumer_idx = cons;
    arg.slot = slot;
    arg.width = edev_camera_read32(edev, base, EDEV_CAMERA_REG_WIDTH);
    arg.height = edev_camera_read32(edev, base, EDEV_CAMERA_REG_HEIGHT);
    arg.frame_bytes = edev_camera_read32(edev, base, EDEV_CAMERA_REG_FRAME_BYTES);
    arg.frame_seq = cons;
    arg.drop_count = edev_camera_read32(edev, base, EDEV_CAMERA_REG_DROP_COUNT);
    arg.status = status;
    arg.dev_addr = ring_base + (u64)slot * stride;

    /* Keep the camera claim serialized until the D2H copy has completed.
     * Do not advance CONSUMER_IDX before DMA completion: doing so would allow
     * the capture core to recycle/overwrite the DDR slot while PCIe is still
     * reading it.  This kernel-assisted path is intended for frame-ready IRQ
     * driven capture; one in-kernel frame DMA at a time is safe and still avoids
     * the userspace producer_idx polling loop.
     */
    ret = edev_submit_buffer_sync(ctx, buf, arg.qid, EDEV_DMA_DIR_D2H,
                                  arg.flags, arg.buf_offset, arg.len,
                                  arg.dev_addr, &arg.op_id, NULL,
                                  &arg.desc_count, true, &cmpl);
    if (!ret && edev_completion_requires_exact_len(edev, arg.qid) && cmpl.bytes_done != arg.len)
        ret = -EIO;
    if (!ret)
        edev_camera_write32(edev, base, EDEV_CAMERA_REG_CONSUMER_IDX, cons + 1u);
    mutex_unlock(&edev->camera_lock);

out_deactive:
    edev_user_buffer_active_dec(ctx, buf);
out_copy:
    arg.result = ret;
    if (copy_to_user(uarg, &arg, sizeof(arg)))
        return -EFAULT;
    return ret;
}

static long edev_chr_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct edev_file_ctx *ctx = filp->private_data;
    struct edev_dev *edev = ctx->edev;
    void __user *uarg = (void __user *)arg;
    u32 val;

    switch (cmd) {
    case EDEV_IOC_SET_IRQ_MODE:
        if (copy_from_user(&val, uarg, sizeof(val)))
            return -EFAULT;
        if (val && edev->irq_vector_count < 1)
            return -EINVAL;
        if (!edev_hw_mmio_accessible(edev))
            return -ENODEV;
        edev->irq_mode = !!val;
        edev_write32(edev, EDEV_REG_G_MODE,
                     EDEV_MODE_SGDMA |
                     ((edev->irq_mode || edev->camera_irq_mode) ? EDEV_MODE_IRQ : 0));
        edev_write32(edev, EDEV_REG_G_IRQ_ENABLE,
                     (edev->irq_mode ? (EDEV_IRQ_CMPL | EDEV_IRQ_ERROR) : 0) |
                     (edev->camera_irq_mode ? EDEV_IRQ_CAMERA_FRAME : 0));
        for (val = 0; val < edev->queue_count; val++)
            edev_q_write32(edev, val, EDEV_QREG_IRQ_ENABLE,
                           edev->irq_mode ? (EDEV_IRQ_CMPL | EDEV_IRQ_ERROR) : 0);
        edev_write32(edev, EDEV_REG_G_QUEUE_COUNT, edev->queue_count);
        return 0;
    case EDEV_IOC_RESET_ENGINE:
        down_write(&edev->submit_rwsem);
        mutex_lock(&edev->sched_lock);
        if (!edev_hw_mmio_accessible(edev)) {
            mutex_unlock(&edev->sched_lock);
            up_write(&edev->submit_rwsem);
            return -ENODEV;
        }
        (void)edev_wait_hw_idle(edev, 100);
        if (READ_ONCE(edev->hw_offline)) {
            mutex_unlock(&edev->sched_lock);
            up_write(&edev->submit_rwsem);
            return -ENODEV;
        }
        edev_hw_reset_force(edev);
        for (val = 0; val < edev->queue_count; val++)
            edev_reset_queue_sw_state(&edev->queues[val]);
        dma_wmb();
        edev_hw_configure(edev);
        atomic_inc(&edev->recovery_epoch);
        wake_up_interruptible(&edev->waitq);
        mutex_unlock(&edev->sched_lock);
        up_write(&edev->submit_rwsem);
        return 0;
    case EDEV_IOC_QUEUE_CONFIG:
        return edev_queue_config(edev, uarg);
    case EDEV_IOC_QUEUE_STATS:
        return edev_queue_stats(edev, uarg);
    case EDEV_IOC_GET_CAPS:
        return edev_get_caps(edev, uarg);
    case EDEV_IOC_REGISTER_BUFFER:
        return edev_register_user_buffer(ctx, uarg);
    case EDEV_IOC_REGISTER_DMABUF:
        return edev_register_dmabuf(ctx, (struct edev_ioc_dmabuf __user *)uarg);
    case EDEV_IOC_CAMERA_WAIT_FRAME:
        return edev_camera_wait_frame(ctx, (struct edev_ioc_camera_wait __user *)uarg);
    case EDEV_IOC_CAMERA_SUBMIT_FRAME:
        return edev_camera_submit_frame(ctx, (struct edev_ioc_camera_submit_frame __user *)uarg);
    case EDEV_IOC_UNREGISTER_BUFFER:
        return edev_unregister_user_buffer(ctx, uarg);
    case EDEV_IOC_SUBMIT_BUFFER:
        return edev_submit_registered(ctx, uarg);
    case EDEV_IOC_REAP_CMPL:
        return edev_reap_cached(ctx, (struct edev_ioc_dma_cmpl __user *)uarg);
    case EDEV_IOC_SUBMIT_2D:
        return edev_submit_2d(ctx, (struct edev_ioc_dma_submit_2d __user *)uarg);
    case EDEV_IOC_SUBMIT_BOUNCE:
        return edev_submit_bounce(ctx, uarg);
    case EDEV_IOC_SUBMIT_BATCH:
        return edev_submit_batch(ctx, uarg);
    case EDEV_IOC_REAP_BATCH:
        return edev_reap_cached_batch(ctx, (struct edev_ioc_reap_batch __user *)uarg);
    case RECONIC_DMA_IOC_ALLOC_COHERENT:
    case RECONIC_DMA_IOC_ALLOC_COHERENT_LEGACY:
        return edev_alloc_coherent_compat(ctx, (struct reconic_dma_alloc_req __user *)uarg);
    case RECONIC_DMA_IOC_FREE_COHERENT:
    case RECONIC_DMA_IOC_FREE_COHERENT_LEGACY:
        edev_free_coherent(ctx);
        return 0;
    case RECONIC_DMA_IOC_BAR2_READ:
        return edev_bar2_compat(ctx, (struct reconic_bar2_access __user *)uarg, false);
    case RECONIC_DMA_IOC_BAR2_WRITE:
        return edev_bar2_compat(ctx, (struct reconic_bar2_access __user *)uarg, true);
    default:
        return -ENOTTY;
    }
}

static __poll_t edev_chr_poll(struct file *filp, poll_table *wait)
{
    struct edev_file_ctx *ctx = filp->private_data;
    struct edev_dev *edev = ctx->edev;

    if (ctx->queue_file && edev_queue_valid(edev, ctx->qid))
        poll_wait(filp, &edev->queues[ctx->qid].waitq, wait);
    else {
        poll_wait(filp, &edev->waitq, wait);
        poll_wait(filp, &edev->camera_waitq, wait);
    }

    /* Sync-only ABI: submit ioctls return completion status; there is no
     * userspace-visible async completion queue.
     */
    return EPOLLOUT | EPOLLWRNORM;
}

static int edev_chr_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct edev_file_ctx *ctx = filp->private_data;
    struct edev_dev *edev = ctx->edev;
    unsigned long size = vma->vm_end - vma->vm_start;
    resource_size_t bar_start, bar_len;
    unsigned long pfn;

    if (ctx->coh_cpu) {
        if ((vma->vm_end - vma->vm_start) > ctx->coh_size || vma->vm_pgoff)
            return -EINVAL;
        return dma_mmap_coherent(edev->dev, vma, ctx->coh_cpu,
                                 ctx->coh_dma, ctx->coh_size);
    }
    if (!edev->bar[2])
        return -ENODEV;
    bar_start = pci_resource_start(edev->pdev, 2);
    bar_len = pci_resource_len(edev->pdev, 2);
    if ((vma->vm_pgoff << PAGE_SHIFT) + size > bar_len)
        return -EINVAL;
    pfn = (bar_start >> PAGE_SHIFT) + vma->vm_pgoff;
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
    vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
#else
    vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
#endif
    return remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot);
}

static int edev_check_dev_range(loff_t pos, size_t count)
{
    if (pos < 0)
        return -EINVAL;
    if (dev_mem_size && ((u64)pos > dev_mem_size || count > dev_mem_size - (u64)pos))
        return -EINVAL;
    return 0;
}

static size_t edev_effective_rw_chunk(void)
{
    size_t chunk = rw_chunk_bytes ? rw_chunk_bytes : EDEV_DEFAULT_RW_CHUNK_BYTES;

    if (max_registered_bytes && chunk > max_registered_bytes)
        chunk = max_registered_bytes;
    if (!chunk)
        chunk = PAGE_SIZE;
    return chunk;
}

static int edev_sync_dma_user(struct edev_file_ctx *ctx, char __user *ubuf,
                              size_t count, loff_t pos, u32 dir)
{
    struct edev_user_buffer *buf = NULL;
    struct edev_cmpl_cache cmpl;
    int ret;

    ret = edev_check_dev_range(pos, count);
    if (ret)
        return ret;
    ret = edev_pin_user_buffer(ctx, (unsigned long)ubuf, count, dir, &buf);
    if (ret)
        return ret;

    ret = edev_submit_buffer_sync(ctx, buf, ctx->qid, dir, 0, 0, count, (u64)pos,
                                  NULL, NULL, NULL, false, &cmpl);
    if (!ret && cmpl.bytes_done != count &&
        edev_completion_requires_exact_len(ctx->edev, ctx->qid))
        ret = -EIO;
    edev_release_user_buffer(ctx->edev, buf);
    return ret;
}

static ssize_t edev_queue_rw(struct file *filp, char __user *ubuf,
                             size_t count, loff_t *ppos, u32 dir)
{
    struct edev_file_ctx *ctx = filp->private_data;
    size_t max_chunk = edev_effective_rw_chunk();
    size_t done = 0;
    int ret = 0;

    if (!ctx || !ctx->queue_file || !edev_queue_valid(ctx->edev, ctx->qid))
        return -EINVAL;
    if (!count)
        return 0;

    while (done < count) {
        size_t chunk = min_t(size_t, count - done, max_chunk);

        ret = edev_sync_dma_user(ctx, ubuf + done, chunk, *ppos + done, dir);
        if (ret == -EMSGSIZE && chunk > PAGE_SIZE) {
            max_chunk = max_t(size_t, PAGE_SIZE, chunk >> 1);
            continue;
        }
        if (ret)
            break;
        done += chunk;
    }
    if (done) {
        *ppos += done;
        return done;
    }
    return ret;
}

static ssize_t edev_chr_read(struct file *filp, char __user *ubuf, size_t count, loff_t *ppos)
{
    return edev_queue_rw(filp, ubuf, count, ppos, EDEV_DMA_DIR_D2H);
}

static ssize_t edev_chr_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
    return edev_queue_rw(filp, (char __user *)ubuf, count, ppos, EDEV_DMA_DIR_H2D);
}

static loff_t edev_chr_llseek(struct file *filp, loff_t off, int whence)
{
    loff_t newpos;

    switch (whence) {
    case SEEK_SET:
        newpos = off;
        break;
    case SEEK_CUR:
        newpos = filp->f_pos + off;
        break;
    case SEEK_END:
        if (!dev_mem_size)
            return -EINVAL;
        newpos = (loff_t)dev_mem_size + off;
        break;
    default:
        return -EINVAL;
    }
    if (newpos < 0)
        return -EINVAL;
    if (dev_mem_size && (u64)newpos > dev_mem_size)
        return -EINVAL;
    filp->f_pos = newpos;
    return newpos;
}

static int edev_chr_open(struct inode *inode, struct file *filp)
{
    struct edev_dev *edev = container_of(inode->i_cdev, struct edev_dev, cdev);
    struct edev_file_ctx *ctx;
    unsigned int minor_off;

    if (iminor(inode) < MINOR(edev->devt))
        return -ENODEV;
    minor_off = iminor(inode) - MINOR(edev->devt);
    if (minor_off > edev->queue_count)
        return -ENODEV;

    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;
    ctx->edev = edev;
    ctx->queue_file = minor_off != 0;
    ctx->qid = ctx->queue_file ? minor_off - 1 : 0;
    if (ctx->queue_file && !edev_queue_valid(edev, ctx->qid)) {
        kfree(ctx);
        return -ENODEV;
    }
    mutex_init(&ctx->buf_lock);
    INIT_LIST_HEAD(&ctx->buffers);
    filp->private_data = ctx;
    return 0;
}

static int edev_chr_release(struct inode *inode, struct file *filp)
{
    struct edev_file_ctx *ctx = filp->private_data;
    struct edev_user_buffer *buf, *tmp;

    if (!ctx)
        return 0;
    mutex_lock(&ctx->buf_lock);
    list_for_each_entry_safe(buf, tmp, &ctx->buffers, node) {
        if (buf->active)
            dev_warn(ctx->edev->dev, "releasing active DMA buffer id=%u\n", buf->id);
        list_del(&buf->node);
        ctx->buffer_count--;
        mutex_unlock(&ctx->buf_lock);
        edev_release_user_buffer(ctx->edev, buf);
        mutex_lock(&ctx->buf_lock);
    }
    mutex_unlock(&ctx->buf_lock);
    edev_free_coherent(ctx);
    kfree(ctx);
    (void)inode;
    return 0;
}

static const struct file_operations edev_fops = {
    .owner = THIS_MODULE,
    .open = edev_chr_open,
    .release = edev_chr_release,
    .read = edev_chr_read,
    .write = edev_chr_write,
    .llseek = edev_chr_llseek,
    .poll = edev_chr_poll,
    .mmap = edev_chr_mmap,
    .unlocked_ioctl = edev_chr_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = edev_chr_ioctl,
#endif
};

static irqreturn_t edev_irq_handler(int irq, void *data)
{
    struct edev_irq_vector *iv = data;
    struct edev_dev *edev = iv ? iv->edev : NULL;
    u32 status = 0;
    u32 qid;
    bool handled = false;

    if (!edev)
        return IRQ_NONE;

    if (edev_hw_mmio_accessible(edev)) {
        status = edev_read32(edev, EDEV_REG_G_IRQ_STATUS);
        if (status != 0xffffffffu && (status & EDEV_IRQ_CAMERA_FRAME)) {
            /* Write-1-to-clear the capture-core frame-ready bit.  This wakeup
             * is the only interrupt used by the default camera path; DMA queue
             * completion still uses coherent writeback polling when irq_mode=0.
             */
            edev_write32(edev, EDEV_REG_G_IRQ_STATUS, EDEV_IRQ_CAMERA_FRAME);
            atomic64_inc(&edev->camera_irq_count);
            wake_up_interruptible(&edev->camera_waitq);
            handled = true;
        }
        if (status != 0xffffffffu && edev->irq_mode &&
            (status & (EDEV_IRQ_CMPL | EDEV_IRQ_ERROR))) {
            edev_write32(edev, EDEV_REG_G_IRQ_STATUS,
                         status & (EDEV_IRQ_CMPL | EDEV_IRQ_ERROR));
        }
    }

    if (edev->irq_mode) {
        if (READ_ONCE(shared_irq_vector) || edev->irq_vector_count == 1) {
            for (qid = 0; qid < edev->queue_count; qid++)
                wake_up_interruptible(&edev->queues[qid].waitq);
        } else if (iv->vector < edev->queue_count) {
            wake_up_interruptible(&edev->queues[iv->vector].waitq);
        }
        wake_up_interruptible(&edev->waitq);
        handled = true;
    }

    (void)irq;
    return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int edev_register_chrdev(struct edev_dev *edev)
{
    int id, ret;
    u32 q;

    id = ida_alloc(&edev_ida, GFP_KERNEL);
    if (id < 0)
        return id;
    if (id >= EDEV_MAX_DEVICES) {
        ida_free(&edev_ida, id);
        return -ENOSPC;
    }

    edev->instance_id = id;
    edev->devt = MKDEV(MAJOR(edev_devt_base), id * EDEV_MINOR_STRIDE);
    edev->devt_count = edev->queue_count + 1;
    edev->queue_chardevs = kcalloc(edev->queue_count, sizeof(*edev->queue_chardevs), GFP_KERNEL);
    if (!edev->queue_chardevs) {
        ret = -ENOMEM;
        goto err_ida;
    }

    cdev_init(&edev->cdev, &edev_fops);
    edev->cdev.owner = THIS_MODULE;
    ret = cdev_add(&edev->cdev, edev->devt, edev->devt_count);
    if (ret)
        goto err_qdevs;

    edev->chardev = device_create(edev_class, edev->dev, edev->devt, NULL, "edev%u", id);
    if (IS_ERR(edev->chardev)) {
        ret = PTR_ERR(edev->chardev);
        edev->chardev = NULL;
        goto err_cdev;
    }

    for (q = 0; q < edev->queue_count; q++) {
        edev->queue_chardevs[q] = device_create(edev_class, edev->dev, edev->devt + 1 + q, NULL,
                                                "edev%u_q%u", id, q);
        if (IS_ERR(edev->queue_chardevs[q])) {
            ret = PTR_ERR(edev->queue_chardevs[q]);
            edev->queue_chardevs[q] = NULL;
            goto err_queue_devices;
        }
    }
    return 0;

err_queue_devices:
    while (q > 0) {
        q--;
        if (edev->queue_chardevs[q])
            device_destroy(edev_class, edev->devt + 1 + q);
    }
    if (edev->chardev)
        device_destroy(edev_class, edev->devt);
    edev->chardev = NULL;
err_cdev:
    cdev_del(&edev->cdev);
err_qdevs:
    kfree(edev->queue_chardevs);
    edev->queue_chardevs = NULL;
err_ida:
    ida_free(&edev_ida, id);
    return ret;
}

static void edev_unregister_chrdev(struct edev_dev *edev)
{
    u32 q;

    if (edev->queue_chardevs) {
        for (q = 0; q < edev->queue_count; q++) {
            if (edev->queue_chardevs[q])
                device_destroy(edev_class, edev->devt + 1 + q);
        }
        kfree(edev->queue_chardevs);
        edev->queue_chardevs = NULL;
    }
    if (edev->chardev) {
        device_destroy(edev_class, edev->devt);
        edev->chardev = NULL;
    }
    cdev_del(&edev->cdev);
    ida_free(&edev_ida, edev->instance_id);
}

static int edev_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct edev_dev *edev;
    int ret, vectors, i;

    if (!is_power_of_2(ring_size) || ring_size < 16 || ring_size > EDEV_MAX_RING_SIZE)
        return -EINVAL;
    if (!queue_count || queue_count > EDEV_MAX_QUEUES)
        return -EINVAL;
    if (st_queue_count > queue_count)
        return -EINVAL;

    edev = devm_kzalloc(&pdev->dev, sizeof(*edev), GFP_KERNEL);
    if (!edev)
        return -ENOMEM;
    edev->pdev = pdev;
    edev->dev = &pdev->dev;
    if (st_queue_count > queue_count)
        st_queue_count = queue_count;
    if (st_h2d_queue_count > st_queue_count)
        st_h2d_queue_count = st_queue_count;
    edev->queue_count = queue_count;
    edev->irq_mode = !!irq_mode;
    edev->camera_irq_mode = !!camera_irq_mode;
    mutex_init(&edev->sched_lock);
    mutex_init(&edev->camera_lock);
    init_rwsem(&edev->submit_rwsem);
    init_waitqueue_head(&edev->waitq);
    init_waitqueue_head(&edev->camera_waitq);
    atomic64_set(&edev->camera_irq_count, 0);
    atomic64_set(&edev->camera_wait_wake_count, 0);
    atomic64_set(&edev->next_op_id, 0);
    atomic_set(&edev->recovery_epoch, 0);
    edev->hw_offline = false;
    pci_set_drvdata(pdev, edev);

    ret = pci_enable_device_mem(pdev);
    if (ret)
        return ret;
    ret = pci_request_regions(pdev, DRIVER_NAME);
    if (ret)
        goto err_disable;
    pci_set_master(pdev);
    ret = edev_set_dma_mask(edev);
    if (ret)
        goto err_regions;
    ret = edev_enable_pcie_requester_caps(edev);
    if (ret)
        goto err_regions;
    ret = edev_map_bars(edev);
    if (ret)
        goto err_regions;
    edev_read_hw_limits(edev);
    ret = edev_alloc_queues(edev);
    if (ret)
        goto err_bars;

    /* Allocate MSI-X vectors whenever requested, even if irq_mode starts off.
     * This lets userspace switch between polling and completion-interrupt wait
     * paths via EDEV_IOC_SET_IRQ_MODE without reloading the driver.  IRQs stay
     * disabled in the SGDMA global/queue masks until irq_mode is enabled.
     */
    if (irq_vectors) {
        unsigned int requested_vectors = irq_vectors ? irq_vectors : (READ_ONCE(shared_irq_vector) ? 1 : edev->queue_count);

        if ((requested_vectors != 1 && requested_vectors != edev->queue_count) ||
            requested_vectors > EDEV_IRQ_VECTOR_COUNT) {
            dev_err(edev->dev, "MSI-X supports either one shared vector or queue_count vectors <= %u\n",
                    EDEV_IRQ_VECTOR_COUNT);
            ret = -EINVAL;
            goto err_queues;
        }
        vectors = pci_alloc_irq_vectors(pdev, requested_vectors, requested_vectors, PCI_IRQ_MSIX);
        if (vectors < 0) {
            if (irq_fallback_poll) {
                dev_warn(edev->dev,
                         "MSI-X allocation failed (%d); continuing in polling mode\n",
                         vectors);
                edev->irq_mode = false;
                edev->camera_irq_mode = false;
                vectors = 0;
                goto irq_ready;
            }
            ret = vectors;
            goto err_queues;
        }
        edev->irq_vec = kcalloc(vectors, sizeof(*edev->irq_vec), GFP_KERNEL);
        if (!edev->irq_vec) {
            ret = -ENOMEM;
            goto err_vectors;
        }
        edev->irq_vector_count = vectors;
        for (i = 0; i < vectors; i++) {
            edev->irq_vec[i].edev = edev;
            edev->irq_vec[i].vector = i;
            ret = pci_request_irq(pdev, i, edev_irq_handler, NULL, &edev->irq_vec[i], DRIVER_NAME);
            if (ret)
                goto err_irq_loop;
        }
    }

irq_ready:
    ret = edev_register_chrdev(edev);
    if (ret)
        goto err_irq;

    edev_hw_reset_force(edev);
    edev_hw_configure(edev);
    ret = reconic_netdev_register(edev, pdev, &edev->netdev);
    if (ret)
        goto err_chrdev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0)
    pcie_print_link_status(pdev);
#endif
    dev_info(edev->dev, "probe complete sync-only: queues=%u ST_TX=0..%u ST_RX=%u..%u MM=%u..%u ring=%u irq_vectors=%d irq_mode=%u\n",
             edev->queue_count,
             st_h2d_queue_count ? st_h2d_queue_count - 1 : 0,
             st_h2d_queue_count,
             st_queue_count ? st_queue_count - 1 : 0,
             st_queue_count,
             edev->queue_count ? edev->queue_count - 1 : 0,
             ring_size, edev->irq_vector_count, edev->irq_mode);
    (void)id;
    return 0;

err_chrdev:
    edev_write32(edev, EDEV_REG_G_IRQ_ENABLE, 0);
    edev_write32(edev, EDEV_REG_G_CTRL, 0);
    edev_unregister_chrdev(edev);
err_irq_loop:
err_irq:
    while (edev->irq_vector_count > 0) {
        edev->irq_vector_count--;
        pci_free_irq(pdev, edev->irq_vector_count, &edev->irq_vec[edev->irq_vector_count]);
    }
    kfree(edev->irq_vec);
    edev->irq_vec = NULL;
err_vectors:
    pci_free_irq_vectors(pdev);
err_queues:
    edev_free_queues(edev);
err_bars:
    edev_unmap_bars(edev);
err_regions:
    pci_release_regions(pdev);
err_disable:
    pci_clear_master(pdev);
    pci_disable_device(pdev);
    return ret;
}

static void edev_remove(struct pci_dev *pdev)
{
    struct edev_dev *edev = pci_get_drvdata(pdev);

    if (!edev)
        return;
    if (!READ_ONCE(edev->hw_offline)) {
        edev_write32(edev, EDEV_REG_G_IRQ_ENABLE, 0);
        edev_write32(edev, EDEV_REG_G_CTRL, 0);
    }
    reconic_netdev_unregister(edev->netdev);
    edev->netdev = NULL;
    edev_unregister_chrdev(edev);
    while (edev->irq_vector_count > 0) {
        edev->irq_vector_count--;
        pci_free_irq(pdev, edev->irq_vector_count, &edev->irq_vec[edev->irq_vector_count]);
    }
    kfree(edev->irq_vec);
    edev->irq_vec = NULL;
    pci_free_irq_vectors(pdev);
    edev_free_queues(edev);
    edev_unmap_bars(edev);
    pci_release_regions(pdev);
    pci_clear_master(pdev);
    pci_disable_device(pdev);
}

static const struct pci_device_id edev_pci_ids[] = {
    { PCI_DEVICE(0x10ee, 0x9038) },
    { PCI_DEVICE(0x10ee, 0x9148) },
    { PCI_DEVICE(0x10ee, 0x9248) },
    { PCI_DEVICE(0x10ee, 0x9348) },
    { PCI_DEVICE(0x10ee, 0x903f) },
    { PCI_DEVICE(0x10ee, 0x913f) },
    { PCI_DEVICE(0x10ee, 0x923f) },
    { PCI_DEVICE(0x10ee, 0x933f) },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, edev_pci_ids);

static struct pci_driver edev_pci_driver = {
    .name = DRIVER_NAME,
    .id_table = edev_pci_ids,
    .probe = edev_probe,
    .remove = edev_remove,
};

static int __init edev_init(void)
{
    int ret;

    ret = alloc_chrdev_region(&edev_devt_base, 0, EDEV_MAX_DEVICES * EDEV_MINOR_STRIDE, DRIVER_NAME);
    if (ret)
        return ret;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    edev_class = class_create(DRIVER_NAME);
#else
    edev_class = class_create(THIS_MODULE, DRIVER_NAME);
#endif
    if (IS_ERR(edev_class)) {
        ret = PTR_ERR(edev_class);
        unregister_chrdev_region(edev_devt_base, EDEV_MAX_DEVICES * EDEV_MINOR_STRIDE);
        return ret;
    }
    ret = pci_register_driver(&edev_pci_driver);
    if (ret) {
        class_destroy(edev_class);
        unregister_chrdev_region(edev_devt_base, EDEV_MAX_DEVICES * EDEV_MINOR_STRIDE);
    }
    return ret;
}

static void __exit edev_exit(void)
{
    pci_unregister_driver(&edev_pci_driver);
    class_destroy(edev_class);
    unregister_chrdev_region(edev_devt_base, EDEV_MAX_DEVICES * EDEV_MINOR_STRIDE);
}

module_init(edev_init);
module_exit(edev_exit);

/* dma-buf symbols are exported from the DMA_BUF namespace.  Linux 6.13
 * changed MODULE_IMPORT_NS() to take a string literal; retain Jetson 5.10/5.15
 * source compatibility as well. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
MODULE_IMPORT_NS("DMA_BUF");
#else
MODULE_IMPORT_NS(DMA_BUF);
#endif
