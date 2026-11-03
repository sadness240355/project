#ifndef RN_DMA_POOL_H
#define RN_DMA_POOL_H

#include <stddef.h>
#include <stdint.h>
#include <sys/ioctl.h>

/* edev_sgdma_driver.h compatibility ABI, fixed-width on aarch64 and x86_64. */
struct rn_dma_alloc_request {
  uint64_t size;
  uint64_t dma_addr;
  uint64_t mmap_offset;
};
#define RN_DMA_IOC_ALLOC_COHERENT _IOWR('R', 1, struct rn_dma_alloc_request)
_Static_assert(sizeof(struct rn_dma_alloc_request) == 24, "DMA allocation ABI");

struct rn_dma_pool {
  int fd;
  void* cpu;
  uint64_t dma_addr;
  size_t size;
};

/* Publish CPU stores to an external DMA requester before an MMIO doorbell.
 * ARM's outer-shareable domain includes devices outside the CPU cluster. */
static inline void rn_dma_wmb(void) {
#if defined(__aarch64__)
  __asm__ __volatile__("dmb oshst" ::: "memory");
#else
  __sync_synchronize();
#endif
}

static inline void rn_dma_rmb(void) {
#if defined(__aarch64__)
  __asm__ __volatile__("dmb oshld" ::: "memory");
#else
  __sync_synchronize();
#endif
}

/* Only allocate on the same PCI function whose BAR2 will be programmed.
 * Returns -1 with errno on failure; never falls back to raw physical DMA. */
int rn_dma_pool_open(struct rn_dma_pool* pool, const char* device_path,
                     const char* resource2_path, size_t size);
/* Caller MUST quiesce hardware before releasing its DMA mappings. */
void rn_dma_pool_close(struct rn_dma_pool* pool);

#endif
