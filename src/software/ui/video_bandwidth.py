"""One-second DMA/RDMA rate samples, including idle zero-bandwidth intervals."""
from collections import deque


class BandwidthHistory:
    def __init__(self, seconds=60):
        self.samples = deque(maxlen=seconds)
        self.previous = None
        self.session = None
        self.last_rates = (0.0, 0.0)
        self.last_change = (0.0, 0.0)
        self.last_sample_at = None

    def zero(self, now, force=False):
        if force or self.last_sample_at is None or now - self.last_sample_at >= 0.8:
            self.samples.append((0.0, 0.0))
            self.last_sample_at = now
            return True
        return False

    def update(self, now, session, dma_frames, rdma_frames, dma_bytes, rdma_bytes,
               dma_updated=None, rdma_updated=None):
        counts = (dma_frames, rdma_frames)
        if not session or any(value is None for value in counts):
            transition = self.session is not None
            self.previous = None
            self.session = None
            self.last_rates = (0.0, 0.0)
            self.last_change = (0.0, 0.0)
            return self.zero(now, force=transition)
        if session != self.session:
            self.session = session
            self.previous = (now, *counts, dma_updated, rdma_updated)
            self.last_rates = (0.0, 0.0)
            self.last_change = (0.0, 0.0)
            return self.zero(now, force=True)
        if self.previous is None:
            self.previous = (now, *counts, dma_updated, rdma_updated)
            return self.zero(now)
        then, previous_dma, previous_rdma, previous_dma_updated, previous_rdma_updated = self.previous
        elapsed = now - then
        if elapsed < 0.8:
            return False
        self.previous = (now, *counts, dma_updated, rdma_updated)
        if elapsed > 5 or dma_frames < previous_dma or rdma_frames < previous_rdma:
            self.last_rates = (0.0, 0.0)
            self.last_change = (0.0, 0.0)
            return self.zero(now, force=True)
        # A peer status poll can repeat a stale sample then catch up. Use each
        # producer's own update interval to avoid a false bandwidth spike.
        dma_elapsed = (dma_updated - previous_dma_updated
                       if dma_updated is not None and previous_dma_updated is not None
                       and dma_updated > previous_dma_updated else elapsed)
        rdma_elapsed = (rdma_updated - previous_rdma_updated
                        if rdma_updated is not None and previous_rdma_updated is not None
                        and rdma_updated > previous_rdma_updated else elapsed)
        rates = []
        changed_at = list(self.last_change)
        for index, (current, previous, size, period, updated, prior_updated) in enumerate((
                (dma_frames, previous_dma, dma_bytes, dma_elapsed, dma_updated, previous_dma_updated),
                (rdma_frames, previous_rdma, rdma_bytes, rdma_elapsed, rdma_updated, previous_rdma_updated))):
            if current > previous:
                rates.append((current - previous) * size / period / 1048576)
                changed_at[index] = now
            elif updated is not None and updated == prior_updated and now - changed_at[index] <= 2.5:
                rates.append(self.last_rates[index])
            else:
                rates.append(0.0)
        self.last_rates = tuple(rates)
        self.last_change = tuple(changed_at)
        self.samples.append(self.last_rates)
        self.last_sample_at = now
        return True

    def update_totals(self, now, session, dma_total, rdma_total,
                      dma_updated=None, rdma_updated=None):
        """Sample completed DMA/RDMA byte counters; never infer stress traffic from frames."""
        if not session or dma_total is None or rdma_total is None:
            transition = self.session is not None
            self.previous = None
            self.session = None
            self.last_rates = (0.0, 0.0)
            self.last_change = (0.0, 0.0)
            return self.zero(now, force=transition)
        key = ('totals', session)
        if self.session != key or self.previous is None:
            self.session = key
            self.previous = (now, dma_total, rdma_total, dma_updated, rdma_updated)
            self.last_rates = (0.0, 0.0)
            self.last_change = (0.0, 0.0)
            return self.zero(now, force=True)
        then, old_dma, old_rdma, old_dma_updated, old_rdma_updated = self.previous
        if now - then < 0.8:
            return False
        self.previous = (now, dma_total, rdma_total, dma_updated, rdma_updated)
        if now - then > 5 or dma_total < old_dma or rdma_total < old_rdma:
            self.last_rates = (0.0, 0.0)
            self.last_change = (0.0, 0.0)
            return self.zero(now, force=True)
        values = []
        changed_at = list(self.last_change)
        for index, (current, previous, updated, old_updated) in enumerate((
            (dma_total, old_dma, dma_updated, old_dma_updated),
            (rdma_total, old_rdma, rdma_updated, old_rdma_updated))):
            if current > previous:
                period = updated - old_updated if updated is not None and old_updated is not None and updated > old_updated else now - then
                values.append((current - previous) / max(period, .001) / 1048576)
                changed_at[index] = now
            elif updated is not None and updated == old_updated and now - changed_at[index] <= 2.5:
                values.append(self.last_rates[index])
            else:
                values.append(0.0)
        self.last_rates = tuple(values)
        self.last_change = tuple(changed_at)
        self.samples.append(self.last_rates)
        self.last_sample_at = now
        return True
