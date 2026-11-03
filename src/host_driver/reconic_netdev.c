// SPDX-License-Identifier: MIT
/* ReconIC network interface layered on the camera SGDMA PCI core.
 *
 * This file deliberately has no pci_driver.  BARs, MSI-X vectors and all DMA
 * queues are owned by edev_sgdma_driver.c; the netdev only claims the fixed ST
 * queue ranges through the in-kernel asynchronous API.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>
#include <linux/version.h>
#include "edev_sgdma_driver.h"
#include "reconic_netdev.h"

#define RECONIC_NET_VERSION "1.0-unified"
#define RECONIC_RX_BUF_MIN 2048u
#define RECONIC_CMAC_PORT 0u

#define SYSCFG_OFFSET_SHELL_RESET 0x000cu
#define SYSCFG_OFFSET_SHELL_STATUS 0x0010u
#define SYSCFG_SHELL_RESET_CMAC0 BIT(4)
#define SYSCFG_SHELL_STATUS_CMAC0_DONE BIT(4)
#define RECONIC_DMA_QCONF 0x1000u

#define CMAC_BASE(i) ((i) ? 0xc000u : 0x8000u)
#define CMAC_CONF_TX_1(i) (CMAC_BASE(i) + 0x000cu)
#define CMAC_CONF_RX_1(i) (CMAC_BASE(i) + 0x0014u)
#define CMAC_CONF_TX_FC_CTRL_1(i) (CMAC_BASE(i) + 0x0030u)
#define CMAC_CONF_TX_FC_RFRH_1(i) (CMAC_BASE(i) + 0x0034u)
#define CMAC_CONF_TX_FC_RFRH_2(i) (CMAC_BASE(i) + 0x0038u)
#define CMAC_CONF_TX_FC_RFRH_3(i) (CMAC_BASE(i) + 0x003cu)
#define CMAC_CONF_TX_FC_RFRH_4(i) (CMAC_BASE(i) + 0x0040u)
#define CMAC_CONF_TX_FC_RFRH_5(i) (CMAC_BASE(i) + 0x0044u)
#define CMAC_CONF_TX_FC_QNTA_1(i) (CMAC_BASE(i) + 0x0048u)
#define CMAC_CONF_TX_FC_QNTA_2(i) (CMAC_BASE(i) + 0x004cu)
#define CMAC_CONF_TX_FC_QNTA_3(i) (CMAC_BASE(i) + 0x0050u)
#define CMAC_CONF_TX_FC_QNTA_4(i) (CMAC_BASE(i) + 0x0054u)
#define CMAC_CONF_TX_FC_QNTA_5(i) (CMAC_BASE(i) + 0x0058u)
#define CMAC_CONF_RX_FC_CTRL_1(i) (CMAC_BASE(i) + 0x0084u)
#define CMAC_CONF_RX_FC_CTRL_2(i) (CMAC_BASE(i) + 0x0088u)
#define CMAC_RSFEC_IND_CORRECTION(i) (CMAC_BASE(i) + 0x1000u)
#define CMAC_RSFEC_ENABLE(i) (CMAC_BASE(i) + 0x107cu)
#define CMAC_STAT_RX_STATUS(i) (CMAC_BASE(i) + 0x0204u)

#define CMAC_PAUSE_ENABLE_ALL GENMASK(8, 0)
#define CMAC_RX_FC_CTRL1 \
    (CMAC_PAUSE_ENABLE_ALL | BIT(10) | BIT(11) | BIT(12) | BIT(13) | GENMASK(23, 15))
#define CMAC_RX_FC_CTRL2 0x0001c631u

static bool netdev_enable = true;
module_param(netdev_enable, bool, 0444);
MODULE_PARM_DESC(netdev_enable, "register ReconIC netdev on the unified SGDMA device");

static bool cmac_on_open = true;
module_param(cmac_on_open, bool, 0444);
MODULE_PARM_DESC(cmac_on_open, "enable and align CMAC when the network interface opens");

static bool rsfec_en;
module_param(rsfec_en, bool, 0444);
MODULE_PARM_DESC(rsfec_en, "enable CMAC RS-FEC");

static unsigned int net_poll_us = 50;
module_param(net_poll_us, uint, 0644);
MODULE_PARM_DESC(net_poll_us, "ST completion polling delay in microseconds");

/* Accepted for old dma_driver load scripts.  The RTL has one ST writeback per
 * queue, so exactly one RX buffer is posted at a time in the unified driver. */
static unsigned int rx_desc_count = 1;
module_param(rx_desc_count, uint, 0444);
MODULE_PARM_DESC(rx_desc_count, "compatibility parameter; effective ST RX depth is fixed at one");

static unsigned int poll_interval_ms;
module_param(poll_interval_ms, uint, 0644);
MODULE_PARM_DESC(poll_interval_ms, "legacy network polling interval; 0 uses net_poll_us");

static unsigned int tx_backlog = 1024;
module_param(tx_backlog, uint, 0644);
MODULE_PARM_DESC(tx_backlog, "maximum queued packets per TX queue");

struct reconic_netdev;

struct reconic_tx_queue
{
    struct reconic_netdev *rdev;
    struct delayed_work work;
    struct sk_buff_head pending;
    struct sk_buff *active_skb;
    dma_addr_t active_dma;
    u32 active_len;
    u32 qid;
    u32 index;
};

struct reconic_rx_queue
{
    struct reconic_netdev *rdev;
    struct delayed_work work;
    struct sk_buff *active_skb;
    dma_addr_t active_dma;
    u32 active_len;
    u32 qid;
    u32 index;
};

struct reconic_netdev
{
    struct edev_dev *edev;
    struct pci_dev *pdev;
    struct net_device *ndev;
    struct reconic_tx_queue *txq;
    struct reconic_rx_queue *rxq;
    u32 tx_count;
    u32 rx_count;
    bool running;
    atomic64_t tx_packets;
    atomic64_t tx_bytes;
    atomic64_t tx_dropped;
    atomic64_t tx_errors;
    atomic64_t rx_packets;
    atomic64_t rx_bytes;
    atomic64_t rx_dropped;
    atomic64_t rx_errors;
};

static unsigned long reconic_poll_delay(void)
{
    if (poll_interval_ms)
        return max_t(unsigned long, 1, msecs_to_jiffies(poll_interval_ms));
    return net_poll_us ? max_t(unsigned long, 1, usecs_to_jiffies(net_poll_us)) : 0;
}

static int reconic_bar_write(struct reconic_netdev *rdev, u32 off, u32 val)
{
    return edev_bar2_write32(rdev->edev, off, val);
}

static int reconic_bar_read(struct reconic_netdev *rdev, u32 off, u32 *val)
{
    return edev_bar2_read32(rdev->edev, off, val);
}

static void reconic_cmac_disable(struct reconic_netdev *rdev)
{
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_1(RECONIC_CMAC_PORT), 0);
    (void)reconic_bar_write(rdev, CMAC_CONF_RX_1(RECONIC_CMAC_PORT), 0);
}

static int reconic_cmac_reset(struct reconic_netdev *rdev)
{
    u32 value = 0;
    int i;

    /* CMAC0 occupies shell reset/done slot 4 in the implemented top-level
     * reset vector.  Some legacy comments describe a compacted status map,
     * but system_config_register indexes status with shell_rstn/done directly.
     */
    if (reconic_bar_write(rdev, SYSCFG_OFFSET_SHELL_RESET,
                          SYSCFG_SHELL_RESET_CMAC0))
        return -EIO;
    for (i = 0; i < 100; i++)
    {
        if (!reconic_bar_read(rdev, SYSCFG_OFFSET_SHELL_STATUS, &value) &&
            (value & SYSCFG_SHELL_STATUS_CMAC0_DONE))
            return 0;
        usleep_range(1000, 2000);
    }
    netdev_err(rdev->ndev,
               "CMAC0 reset timed out: shell_status=0x%08x expected_done=0x%08x\n",
               value, SYSCFG_SHELL_STATUS_CMAC0_DONE);
    return -ETIMEDOUT;
}

static int reconic_cmac_enable(struct reconic_netdev *rdev)
{
    u32 port = RECONIC_CMAC_PORT;
    u32 status = 0;
    int i;
    int ret;

    ret = reconic_cmac_reset(rdev);
    if (ret)
        return ret;
    if (rsfec_en)
    {
        if (reconic_bar_write(rdev, CMAC_RSFEC_ENABLE(port), 0x3) ||
            reconic_bar_write(rdev, CMAC_RSFEC_IND_CORRECTION(port), 0x7))
            return -EIO;
    }
    if (reconic_bar_write(rdev, CMAC_CONF_RX_1(port), 0x1) ||
        reconic_bar_write(rdev, CMAC_CONF_TX_1(port), 0x1))
        return -EIO;

    for (i = 0; i < 40; i++)
    {
        if (!reconic_bar_read(rdev, CMAC_STAT_RX_STATUS(port), &status) &&
            (status & 0x3) == 0x3)
            break;
        if (i == 8)
        {
            ret = reconic_cmac_reset(rdev);
            if (ret)
                return ret;
            (void)reconic_bar_write(rdev, CMAC_CONF_RX_1(port), 0x1);
            (void)reconic_bar_write(rdev, CMAC_CONF_TX_1(port), 0x1);
        }
        msleep(50);
    }
    if (i == 40)
    {
        netdev_err(rdev->ndev,
                   "CMAC0 RX alignment timed out: rx_status=0x%08x expected_mask=0x00000003 rsfec=%u\n",
                   status, rsfec_en ? 1 : 0);
        reconic_cmac_disable(rdev);
        return -ETIMEDOUT;
    }

    (void)reconic_bar_write(rdev, CMAC_CONF_TX_1(port), 0x1);
    (void)reconic_bar_write(rdev, CMAC_CONF_RX_FC_CTRL_1(port), CMAC_RX_FC_CTRL1);
    (void)reconic_bar_write(rdev, CMAC_CONF_RX_FC_CTRL_2(port), CMAC_RX_FC_CTRL2);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_QNTA_1(port), 0xffffffffu);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_QNTA_2(port), 0xffffffffu);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_QNTA_3(port), 0xffffffffu);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_QNTA_4(port), 0xffffffffu);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_QNTA_5(port), 0x0000ffffu);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_RFRH_1(port), 0xffffffffu);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_RFRH_2(port), 0xffffffffu);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_RFRH_3(port), 0xffffffffu);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_RFRH_4(port), 0xffffffffu);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_RFRH_5(port), 0x0000ffffu);
    (void)reconic_bar_write(rdev, CMAC_CONF_TX_FC_CTRL_1(port), CMAC_PAUSE_ENABLE_ALL);
    return 0;
}

static void reconic_configure_rx_queues(struct reconic_netdev *rdev)
{
    u32 base = edev_kernel_st_rx_base(rdev->edev);
    u32 value = (base << 16) | rdev->rx_count;

    (void)reconic_bar_write(rdev, RECONIC_DMA_QCONF, value);
}

static void reconic_tx_finish(struct reconic_tx_queue *txq, bool error)
{
    struct reconic_netdev *rdev = txq->rdev;

    dma_unmap_single(&rdev->pdev->dev, txq->active_dma,
                     txq->active_len, DMA_TO_DEVICE);
    if (error)
    {
        atomic64_inc(&rdev->tx_errors);
        atomic64_inc(&rdev->tx_dropped);
        dev_kfree_skb_any(txq->active_skb);
    }
    else
    {
        atomic64_inc(&rdev->tx_packets);
        atomic64_add(txq->active_len, &rdev->tx_bytes);
        dev_consume_skb_any(txq->active_skb);
    }
    txq->active_skb = NULL;
    txq->active_dma = 0;
    txq->active_len = 0;
}

static void reconic_tx_work(struct work_struct *work)
{
    struct reconic_tx_queue *txq =
        container_of(to_delayed_work(work), struct reconic_tx_queue, work);
    struct reconic_netdev *rdev = txq->rdev;
    struct edev_kernel_cmpl cmpl;
    struct sk_buff *skb;
    dma_addr_t dma;
    int ret;

    if (txq->active_skb)
    {
        ret = edev_kernel_async_poll(rdev->edev, txq->qid, &cmpl);
        if (ret == 0)
        {
            schedule_delayed_work(&txq->work, reconic_poll_delay());
            return;
        }
        reconic_tx_finish(txq, ret < 0 || cmpl.error_code ||
                                   (cmpl.status & EDEV_WB_STATUS_ERROR));
    }
    if (!READ_ONCE(rdev->running))
        return;

    skb = skb_dequeue(&txq->pending);
    if (!skb)
    {
        netif_tx_wake_queue(netdev_get_tx_queue(rdev->ndev, txq->index));
        return;
    }
    dma = dma_map_single(&rdev->pdev->dev, skb->data, skb->len, DMA_TO_DEVICE);
    if (dma_mapping_error(&rdev->pdev->dev, dma))
    {
        atomic64_inc(&rdev->tx_errors);
        atomic64_inc(&rdev->tx_dropped);
        dev_kfree_skb_any(skb);
        schedule_delayed_work(&txq->work, 0);
        return;
    }
    ret = edev_kernel_async_submit(rdev->edev, txq->qid, dma, skb->len, true);
    if (ret)
    {
        dma_unmap_single(&rdev->pdev->dev, dma, skb->len, DMA_TO_DEVICE);
        if (ret == -EBUSY || ret == -ENOSPC)
        {
            skb_queue_head(&txq->pending, skb);
            schedule_delayed_work(&txq->work, reconic_poll_delay());
        }
        else
        {
            atomic64_inc(&rdev->tx_errors);
            atomic64_inc(&rdev->tx_dropped);
            dev_kfree_skb_any(skb);
            schedule_delayed_work(&txq->work, 0);
        }
        return;
    }
    txq->active_skb = skb;
    txq->active_dma = dma;
    txq->active_len = skb->len;
    if (skb_queue_len(&txq->pending) < max_t(unsigned int, 2, tx_backlog / 2))
        netif_tx_wake_queue(netdev_get_tx_queue(rdev->ndev, txq->index));
    schedule_delayed_work(&txq->work, reconic_poll_delay());
}

static void reconic_rx_finish(struct reconic_rx_queue *rxq,
                              const struct edev_kernel_cmpl *cmpl, int poll_ret)
{
    struct reconic_netdev *rdev = rxq->rdev;
    struct sk_buff *skb = rxq->active_skb;
    bool error = poll_ret < 0 || cmpl->error_code ||
                 (cmpl->status & (EDEV_WB_STATUS_ERROR |
                                  EDEV_WB_STATUS_TRUNC |
                                  EDEV_WB_STATUS_DROP)) ||
                 !cmpl->bytes || cmpl->bytes > rxq->active_len;

    dma_unmap_single(&rdev->pdev->dev, rxq->active_dma,
                     rxq->active_len, DMA_FROM_DEVICE);
    rxq->active_skb = NULL;
    rxq->active_dma = 0;
    rxq->active_len = 0;
    if (error || !READ_ONCE(rdev->running))
    {
        if (error)
            atomic64_inc(&rdev->rx_errors);
        atomic64_inc(&rdev->rx_dropped);
        dev_kfree_skb_any(skb);
        return;
    }
    skb_put(skb, cmpl->bytes);
    skb->protocol = eth_type_trans(skb, rdev->ndev);
    skb->ip_summed = CHECKSUM_NONE;
    skb_record_rx_queue(skb, rxq->index);
    atomic64_inc(&rdev->rx_packets);
    atomic64_add(cmpl->bytes, &rdev->rx_bytes);
    netif_rx(skb);
}

static void reconic_rx_work(struct work_struct *work)
{
    struct reconic_rx_queue *rxq =
        container_of(to_delayed_work(work), struct reconic_rx_queue, work);
    struct reconic_netdev *rdev = rxq->rdev;
    struct edev_kernel_cmpl cmpl = {};
    struct sk_buff *skb;
    dma_addr_t dma;
    u32 len;
    int ret;

    if (rxq->active_skb)
    {
        ret = edev_kernel_async_poll(rdev->edev, rxq->qid, &cmpl);
        if (ret == 0)
        {
            if (READ_ONCE(rdev->running))
                schedule_delayed_work(&rxq->work, reconic_poll_delay());
            return;
        }
        reconic_rx_finish(rxq, &cmpl, ret);
    }
    if (!READ_ONCE(rdev->running))
        return;

    len = max_t(u32, RECONIC_RX_BUF_MIN,
                rdev->ndev->mtu + ETH_HLEN + VLAN_HLEN);
    skb = netdev_alloc_skb_ip_align(rdev->ndev, len);
    if (!skb)
    {
        atomic64_inc(&rdev->rx_dropped);
        schedule_delayed_work(&rxq->work, 1);
        return;
    }
    dma = dma_map_single(&rdev->pdev->dev, skb->data, len, DMA_FROM_DEVICE);
    if (dma_mapping_error(&rdev->pdev->dev, dma))
    {
        atomic64_inc(&rdev->rx_errors);
        dev_kfree_skb_any(skb);
        schedule_delayed_work(&rxq->work, 1);
        return;
    }
    ret = edev_kernel_async_submit(rdev->edev, rxq->qid, dma, len, false);
    if (ret)
    {
        dma_unmap_single(&rdev->pdev->dev, dma, len, DMA_FROM_DEVICE);
        dev_kfree_skb_any(skb);
        if (ret != -EBUSY && ret != -ENOSPC)
            atomic64_inc(&rdev->rx_errors);
        schedule_delayed_work(&rxq->work, reconic_poll_delay());
        return;
    }
    rxq->active_skb = skb;
    rxq->active_dma = dma;
    rxq->active_len = len;
    schedule_delayed_work(&rxq->work, reconic_poll_delay());
}

static netdev_tx_t reconic_start_xmit(struct sk_buff *skb,
                                      struct net_device *ndev)
{
    struct reconic_netdev *rdev = netdev_priv(ndev);
    struct reconic_tx_queue *txq;
    u32 index;

    if (!READ_ONCE(rdev->running) || !netif_carrier_ok(ndev))
    {
        atomic64_inc(&rdev->tx_dropped);
        dev_kfree_skb_any(skb);
        return NETDEV_TX_OK;
    }
    index = skb_get_queue_mapping(skb) % rdev->tx_count;
    txq = &rdev->txq[index];
    if (skb_queue_len(&txq->pending) >= max_t(unsigned int, 2, tx_backlog))
    {
        netif_tx_stop_queue(netdev_get_tx_queue(ndev, index));
        return NETDEV_TX_BUSY;
    }
    if (skb_put_padto(skb, ETH_ZLEN))
    {
        atomic64_inc(&rdev->tx_dropped);
        return NETDEV_TX_OK;
    }
    if (skb_is_nonlinear(skb) && skb_linearize(skb))
    {
        atomic64_inc(&rdev->tx_dropped);
        dev_kfree_skb_any(skb);
        return NETDEV_TX_OK;
    }
    skb_queue_tail(&txq->pending, skb);
    if (skb_queue_len(&txq->pending) >= max_t(unsigned int, 2, tx_backlog))
        netif_tx_stop_queue(netdev_get_tx_queue(ndev, index));
    schedule_delayed_work(&txq->work, 0);
    return NETDEV_TX_OK;
}

static int reconic_open(struct net_device *ndev)
{
    struct reconic_netdev *rdev = netdev_priv(ndev);
    u32 i;
    int ret;

    if (cmac_on_open)
    {
        ret = reconic_cmac_enable(rdev);
        if (ret)
            return ret;
    }
    reconic_configure_rx_queues(rdev);
    WRITE_ONCE(rdev->running, true);
    netif_carrier_on(ndev);
    netif_tx_start_all_queues(ndev);
    for (i = 0; i < rdev->rx_count; i++)
        schedule_delayed_work(&rdev->rxq[i].work, 0);
    for (i = 0; i < rdev->tx_count; i++)
        if (rdev->txq[i].active_skb || !skb_queue_empty(&rdev->txq[i].pending))
            schedule_delayed_work(&rdev->txq[i].work, 0);
    return 0;
}

static int reconic_stop(struct net_device *ndev)
{
    struct reconic_netdev *rdev = netdev_priv(ndev);
    u32 i;

    WRITE_ONCE(rdev->running, false);
    netif_tx_stop_all_queues(ndev);
    netif_carrier_off(ndev);
    for (i = 0; i < rdev->tx_count; i++)
    {
        cancel_delayed_work_sync(&rdev->txq[i].work);
        skb_queue_purge(&rdev->txq[i].pending);
    }
    for (i = 0; i < rdev->rx_count; i++)
        cancel_delayed_work_sync(&rdev->rxq[i].work);
    if (cmac_on_open)
        reconic_cmac_disable(rdev);
    return 0;
}

static int reconic_change_mtu(struct net_device *ndev, int mtu)
{
    if (mtu < ETH_MIN_MTU || mtu > 9000)
        return -EINVAL;
    ndev->mtu = mtu;
    return 0;
}

static void reconic_get_stats64(struct net_device *ndev,
                                struct rtnl_link_stats64 *stats)
{
    struct reconic_netdev *rdev = netdev_priv(ndev);

    stats->tx_packets = atomic64_read(&rdev->tx_packets);
    stats->tx_bytes = atomic64_read(&rdev->tx_bytes);
    stats->tx_dropped = atomic64_read(&rdev->tx_dropped);
    stats->tx_errors = atomic64_read(&rdev->tx_errors);
    stats->rx_packets = atomic64_read(&rdev->rx_packets);
    stats->rx_bytes = atomic64_read(&rdev->rx_bytes);
    stats->rx_dropped = atomic64_read(&rdev->rx_dropped);
    stats->rx_errors = atomic64_read(&rdev->rx_errors);
}

static const struct net_device_ops reconic_netdev_ops = {
    .ndo_open = reconic_open,
    .ndo_stop = reconic_stop,
    .ndo_start_xmit = reconic_start_xmit,
    .ndo_set_mac_address = eth_mac_addr,
    .ndo_validate_addr = eth_validate_addr,
    .ndo_change_mtu = reconic_change_mtu,
    .ndo_get_stats64 = reconic_get_stats64,
};

static void reconic_get_drvinfo(struct net_device *ndev,
                                struct ethtool_drvinfo *info)
{
    strscpy(info->driver, DRIVER_NAME, sizeof(info->driver));
    strscpy(info->version, RECONIC_NET_VERSION, sizeof(info->version));
}

static const struct ethtool_ops reconic_ethtool_ops = {
    .get_drvinfo = reconic_get_drvinfo,
    .get_link = ethtool_op_get_link,
};

static void reconic_release_active(struct reconic_netdev *rdev)
{
    u32 i;

    for (i = 0; i < rdev->tx_count; i++)
    {
        struct reconic_tx_queue *q = &rdev->txq[i];
        edev_kernel_async_cancel(rdev->edev, q->qid);
        if (q->active_skb)
            reconic_tx_finish(q, true);
        skb_queue_purge(&q->pending);
    }
    for (i = 0; i < rdev->rx_count; i++)
    {
        struct reconic_rx_queue *q = &rdev->rxq[i];
        edev_kernel_async_cancel(rdev->edev, q->qid);
        if (q->active_skb)
        {
            dma_unmap_single(&rdev->pdev->dev, q->active_dma,
                             q->active_len, DMA_FROM_DEVICE);
            dev_kfree_skb_any(q->active_skb);
            q->active_skb = NULL;
        }
    }
}

int reconic_netdev_register(struct edev_dev *edev, struct pci_dev *pdev,
                            struct reconic_netdev **out)
{
    struct net_device *ndev;
    struct reconic_netdev *rdev;
    u32 tx_count;
    u32 rx_count;
    u32 rx_base;
    u32 i;
    int ret;

    if (!out)
        return -EINVAL;
    *out = NULL;
    if (!netdev_enable)
        return 0;
    tx_count = edev_kernel_st_tx_count(edev);
    rx_base = edev_kernel_st_rx_base(edev);
    rx_count = edev_kernel_st_rx_count(edev);
    if (!tx_count || !rx_count)
    {
        dev_warn(&pdev->dev, "netdev disabled: unified queue layout has tx=%u rx=%u\n",
                 tx_count, rx_count);
        return 0;
    }

    ndev = alloc_etherdev_mqs(sizeof(*rdev), tx_count, rx_count);
    if (!ndev)
        return -ENOMEM;
    SET_NETDEV_DEV(ndev, &pdev->dev);
    rdev = netdev_priv(ndev);
    rdev->edev = edev;
    rdev->pdev = pdev;
    rdev->ndev = ndev;
    rdev->tx_count = tx_count;
    rdev->rx_count = rx_count;
    rdev->txq = kcalloc(tx_count, sizeof(*rdev->txq), GFP_KERNEL);
    rdev->rxq = kcalloc(rx_count, sizeof(*rdev->rxq), GFP_KERNEL);
    if (!rdev->txq || !rdev->rxq)
    {
        ret = -ENOMEM;
        goto err_free;
    }

    ndev->netdev_ops = &reconic_netdev_ops;
    ndev->ethtool_ops = &reconic_ethtool_ops;
    ndev->features |= NETIF_F_HIGHDMA;
    ndev->hw_features = ndev->features;
    ndev->min_mtu = ETH_MIN_MTU;
    ndev->max_mtu = 9000;
    snprintf(ndev->name, IFNAMSIZ, "reconic%us%uf%u",
             pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
    {
        u8 mac[ETH_ALEN] = {
            0x00, 0x0a, 0x35, 0x00,
            pdev->bus->number, pdev->devfn};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
        eth_hw_addr_set(ndev, mac);
#else
        /*
         * NVIDIA Jetson Linux 5.10中没有eth_hw_addr_set()，
         * 直接复制到net_device的dev_addr。
         */
        ether_addr_copy(ndev->dev_addr, mac);
#endif
    }

    if (rx_desc_count != 1)
        dev_warn(&pdev->dev,
                 "rx_desc_count=%u requested; unified single-WB RTL safely supports one active RX descriptor per queue\n",
                 rx_desc_count);

    for (i = 0; i < tx_count; i++)
    {
        rdev->txq[i].rdev = rdev;
        rdev->txq[i].qid = EDEV_ST_TX_QID_BASE + i;
        rdev->txq[i].index = i;
        skb_queue_head_init(&rdev->txq[i].pending);
        INIT_DELAYED_WORK(&rdev->txq[i].work, reconic_tx_work);
    }
    for (i = 0; i < rx_count; i++)
    {
        rdev->rxq[i].rdev = rdev;
        rdev->rxq[i].qid = rx_base + i;
        rdev->rxq[i].index = i;
        INIT_DELAYED_WORK(&rdev->rxq[i].work, reconic_rx_work);
    }

    ret = register_netdev(ndev);
    if (ret)
        goto err_free;
    netif_carrier_off(ndev);
    *out = rdev;
    dev_info(&pdev->dev,
             "unified ReconIC netdev %s registered: ST TX q0..%u RX q%u..%u, MM camera q%u..%u\n",
             ndev->name, tx_count - 1, rx_base, rx_base + rx_count - 1,
             EDEV_MM_QID_BASE, EDEV_MM_QID_BASE + EDEV_MM_QUEUE_COUNT - 1);
    return 0;

err_free:
    kfree(rdev->txq);
    kfree(rdev->rxq);
    free_netdev(ndev);
    return ret;
}

void reconic_netdev_unregister(struct reconic_netdev *rdev)
{
    if (!rdev)
        return;
    unregister_netdev(rdev->ndev);
    reconic_release_active(rdev);
    kfree(rdev->txq);
    kfree(rdev->rxq);
    free_netdev(rdev->ndev);
}
