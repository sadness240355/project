#include "dma_pool.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

static int same_pci_device(int fd, const char* resource2_path) {
  struct stat st;
  char sysfs_path[128];
  char owner[PATH_MAX];
  char resource[PATH_MAX];
  char* slash;

  if(fstat(fd, &st) != 0) return -1;
  if(!S_ISCHR(st.st_mode)) { errno = ENODEV; return -1; }
  snprintf(sysfs_path, sizeof(sysfs_path), "/sys/dev/char/%u:%u/device",
           major(st.st_rdev), minor(st.st_rdev));
  if(!realpath(sysfs_path, owner) || !realpath(resource2_path, resource))
    return -1;
  slash = strrchr(resource, '/');
  if(!slash || strcmp(slash, "/resource2") != 0) {
    errno = EINVAL;
    return -1;
  }
  *slash = '\0';
  if(strcmp(owner, resource) != 0) { errno = EXDEV; return -1; }
  return 0;
}

int rn_dma_pool_open(struct rn_dma_pool* pool, const char* device_path,
                     const char* resource2_path, size_t size) {
  struct rn_dma_alloc_request req = {.size = size};
  int saved_errno;

  if(!pool || !device_path || !resource2_path || !size || size > UINT32_MAX) {
    errno = EINVAL;
    return -1;
  }
  *pool = (struct rn_dma_pool){.fd = -1};
  pool->fd = open(device_path, O_RDWR | O_CLOEXEC);
  if(pool->fd < 0) return -1;
  if(same_pci_device(pool->fd, resource2_path) != 0) goto fail;
  if(ioctl(pool->fd, RN_DMA_IOC_ALLOC_COHERENT, &req) != 0) goto fail;
  if(req.size < size || req.size > UINT32_MAX || req.mmap_offset != 0 ||
     req.dma_addr > UINT64_MAX - (req.size - 1)) {
    errno = EOVERFLOW;
    goto fail;
  }
  pool->size = (size_t)req.size;
  pool->cpu = mmap(NULL, pool->size, PROT_READ | PROT_WRITE, MAP_SHARED,
                   pool->fd, 0);
  if(pool->cpu == MAP_FAILED) { pool->cpu = NULL; goto fail; }
  pool->dma_addr = req.dma_addr;
  memset(pool->cpu, 0, pool->size);
  return 0;

fail:
  saved_errno = errno;
  /* No hardware has been given this allocation yet. Closing the owner FD
   * releases the driver's per-file coherent allocation. */
  rn_dma_pool_close(pool);
  errno = saved_errno;
  return -1;
}

void rn_dma_pool_close(struct rn_dma_pool* pool) {
  if(pool->cpu) munmap(pool->cpu, pool->size);
  if(pool->fd >= 0) close(pool->fd);
  *pool = (struct rn_dma_pool){.fd = -1};
}
