//==============================================================================
// Copyright (C) 2023, Advanced Micro Devices, Inc. All rights reserved.
// SPDX-License-Identifier: MIT
//
//==============================================================================

/** @file reconic.c
 *  @brief The RecoNIC user-space API library.
 *
 */

#include "reconic.h"

#include <limits.h>

int debug = 0;

char* device = "";

int fpga_fd = -1;

uint64_t get_win_size() {
  //return AXI_BAR_SIZE>>3;
  return AXI_BAR_SIZE;
}

static int get_bar0_resource_path(const char* bar2_path, char* bar0_path,
                                  size_t bar0_path_size) {
  static const char resource2_suffix[] = "/resource2";
  size_t path_len;
  size_t suffix_len = sizeof(resource2_suffix) - 1;

  if((bar2_path == NULL) || (bar0_path == NULL)) {
    errno = EINVAL;
    return -1;
  }

  path_len = strlen(bar2_path);
  if((path_len < suffix_len) ||
     (strcmp(bar2_path + path_len - suffix_len, resource2_suffix) != 0) ||
     (path_len + 1 > bar0_path_size)) {
    errno = EINVAL;
    return -1;
  }

  memcpy(bar0_path, bar2_path, path_len + 1);
  bar0_path[path_len - 1] = '0';
  return 0;
}

uint32_t convert_ip_addr_to_uint(char* ip_addr){
  unsigned char ip_char[4] = {0};
  uint32_t ip;
  if(sscanf(ip_addr, "%hhu.%hhu.%hhu.%hhu",
            &ip_char[0], &ip_char[1], &ip_char[2], &ip_char[3]) != 4) {
    fprintf(stderr, "Error: invalid IPv4 address: %s\n", ip_addr);
    exit(EXIT_FAILURE);
  }
  //fprintf(stderr, "ip = %u.%u.%u.%u\n", ip_char[0], ip_char[1], ip_char[2], ip_char[3]);
  ip = ((uint32_t) ip_char[0] << 24) |
       ((uint32_t) ip_char[1] << 16) |
       ((uint32_t) ip_char[2] << 8) |
       (uint32_t) ip_char[3];
  return ip;
}

struct mac_addr_t convert_mac_addr_str_to_uint(char* mac_addr_str) {
    struct mac_addr_t mac_addr_inst;
    uint32_t mac_addr_lsb;
    uint32_t mac_addr_msb;
    uint32_t mac_addr_array[6] = {0};
    sscanf(mac_addr_str, "%x:%x:%x:%x:%x:%x", &mac_addr_array[0],&mac_addr_array[1],&mac_addr_array[2],&mac_addr_array[3],&mac_addr_array[4],&mac_addr_array[5]);

    fprintf(stderr, "Info: mac_addr_t = %02x:%02x:%02x:%02x:%02x:%02x\n", mac_addr_array[0], mac_addr_array[1], mac_addr_array[2], mac_addr_array[3], mac_addr_array[4], mac_addr_array[5]);

    mac_addr_msb = ((mac_addr_array[0]<<8) | mac_addr_array[1]) & 0x0000ffff;
    mac_addr_lsb = ((mac_addr_array[2]<<24) | (mac_addr_array[3]<<16) | (mac_addr_array[4]<<8) | mac_addr_array[5]) & 0xffffffff;
    mac_addr_inst.mac_lsb = mac_addr_lsb;
    mac_addr_inst.mac_msb = mac_addr_msb;
    return mac_addr_inst;
}

struct mac_addr_t convert_mac_addr_to_uint(unsigned char* mac_addr_char) {
  struct mac_addr_t mac_addr_inst;
  uint32_t mac_addr_lsb;
  uint32_t mac_addr_msb;

  fprintf(stderr, "Info: mac_addr_t = %02x:%02x:%02x:%02x:%02x:%02x\n", mac_addr_char[0], mac_addr_char[1], mac_addr_char[2], mac_addr_char[3], mac_addr_char[4], mac_addr_char[5]);

  mac_addr_msb = ((mac_addr_char[0]<<8) | mac_addr_char[1]) & 0x0000ffff;
  mac_addr_lsb = ((mac_addr_char[2]<<24) | (mac_addr_char[3]<<16) | (mac_addr_char[4]<<8) | mac_addr_char[5]) & 0xffffffff;
  mac_addr_inst.mac_lsb = mac_addr_lsb;
  mac_addr_inst.mac_msb = mac_addr_msb;
  return mac_addr_inst;
}

struct mac_addr_t get_mac_addr_from_str_ip(int sockfd, char* ip_str) {
  struct ifaddrs* ifaddr;
  struct ifaddrs* ifa;
  struct ifreq ifreq_local;
  int family;
  int return_value;
  char tmp_ip[NI_MAXHOST];
  fprintf(stderr, "Info: src_ip = %s\n", (char*) ip_str);
  if(getifaddrs(&ifaddr) == -1) {
    fprintf(stderr, "Error: not able to getifaddrs\n");
    exit(EXIT_FAILURE);
  }
  for(ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
    if(ifa->ifa_addr == NULL) {
      continue;
    }
    family = ifa->ifa_addr->sa_family;
    // Skip interfaces that are not IPv4 addresses
    if(family != AF_INET) {
      continue;
  }

  return_value = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), tmp_ip, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

  // fprintf(stderr, "Info: tmp_ip = %s\n", tmp_ip);

  if(return_value != 0) {
    fprintf(stderr, "Error: getnameinfo() failed with %s\n", gai_strerror(return_value));
    exit(EXIT_FAILURE);
  }

  if (strcmp(tmp_ip, ip_str) == 0) {
    fprintf(stderr, "Info: Found network interface: %s\n", ifa->ifa_name);
    strncpy(ifreq_local.ifr_name, (char* ) ifa->ifa_name, IFNAMSIZ -1);
    ifreq_local.ifr_name[IFNAMSIZ - 1] = '\0';
    if(ioctl(sockfd, SIOCGIFHWADDR, &ifreq_local) != 0) {
      fprintf(stderr, "Error: SIOCGIFHWADDR failed for %s: %s\n",
              ifa->ifa_name, strerror(errno));
      freeifaddrs(ifaddr);
      exit(EXIT_FAILURE);
    }
    // fprintf(stderr, "Getting src_mac address:\n");
    struct mac_addr_t result =
      convert_mac_addr_to_uint((unsigned char* ) ifreq_local.ifr_hwaddr.sa_data);
    freeifaddrs(ifaddr);
    return result;
  }
  }
  freeifaddrs(ifaddr);
  fprintf(stderr, "Cannot find interface with IP address %s\n", ip_str);
  exit(EXIT_FAILURE);
}

uint8_t is_device_address(uint64_t address) {
  if((address & 0xfff0000000000000) == DEVICE_MEM_OFFSET) {
    // Device memory address
    return 1;
  } else {
    // Host memory address
    return 0;
  }
}

/* Used to get the PFN of a virtual address */
unsigned long get_page_frame_number_of_address(void *addr) {
  uint64_t pagemap_entry;
  // Getting the pagemap file for the current process
  FILE *pagemap = fopen("/proc/self/pagemap", "rb");

  if(pagemap == NULL) {
    fprintf(stderr, "Error: cannot open /proc/self/pagemap: %s\n",
            strerror(errno));
    exit(EXIT_FAILURE);
  }

  // Seek to the page that the buffer is on
  unsigned long offset = (unsigned long)addr / getpagesize() * PAGEMAP_LENGTH;
  if(fseek(pagemap, (unsigned long)offset, SEEK_SET) != 0) {
    fprintf(stderr, "Error: Failed to seek pagemap to proper location\n");
    exit(1);
  }

  if(fread(&pagemap_entry, sizeof(pagemap_entry), 1, pagemap) != 1) {
    fprintf(stderr, "Error: failed to get page frame number\n");
    fclose(pagemap);
    exit(EXIT_FAILURE);
  }
  fclose(pagemap);

  if((pagemap_entry & (UINT64_C(1) << 63)) == 0) {
    fprintf(stderr, "Error: requested virtual page is not present\n");
    exit(EXIT_FAILURE);
  }
  pagemap_entry &= UINT64_C(0x007fffffffffffff);
  if(pagemap_entry == 0) {
    fprintf(stderr,
            "Error: pagemap PFN is unavailable; run as root/CAP_SYS_ADMIN\n");
    exit(EXIT_FAILURE);
  }
  return (unsigned long) pagemap_entry;
}

/* This function is used to get the physical address of a buffer. */
uint64_t get_buffer_paddr(void *buffer) {
  // Getting the page frame the buffer is in
  unsigned long page_frame_number = get_page_frame_number_of_address(buffer);

  Debug("Info: get_buffer_paddr - Page frame: 0x%lx\n", page_frame_number);

  // Getting the offset of the buffer into the page
  unsigned int distance_from_page_boundary = (unsigned long)buffer % getpagesize();

  Debug("Info: get_buffer_paddr - distance from page boundary: 0x%x\n", distance_from_page_boundary);

  uint64_t paddr = (uint64_t)(page_frame_number << PAGE_SHIFT) + (uint64_t)distance_from_page_boundary;

  Debug("Info: get_buffer_paddr - Physical address of buffer: 0x%lx\n", paddr);
  return paddr;
}

void config_rn_dev_axib_bdf(struct rn_dev_t* rn_dev, uint32_t high_addr, uint32_t low_addr) {
  uint32_t i;
  uint32_t entry_addr;
  uint64_t host_addr;
  uint64_t host_last_addr;
  uint64_t pcie_base;
  uint64_t window_mask = RN_AXI_BR_WINDOW_MASK;
  uint64_t win_size = get_win_size();

  if((rn_dev == NULL) || (rn_dev->bar0_ctl == NULL)) {
    fprintf(stderr, "Error: BAR0 is not mapped for AXI requester translation\n");
    exit(EXIT_FAILURE);
  }

  host_addr = ((uint64_t) high_addr << 32) | low_addr;
  if((rn_dev->hugepage_map_size == 0) ||
     (host_addr > UINT64_MAX - (rn_dev->hugepage_map_size - 1))) {
    fprintf(stderr, "Error: invalid host DMA memory range\n");
    exit(EXIT_FAILURE);
  }
  host_last_addr = host_addr + rn_dev->hugepage_map_size - 1;
  pcie_base = host_addr & window_mask;

  if((host_last_addr & window_mask) != pcie_base) {
    fprintf(stderr,
            "Error: DMA pool range 0x%016" PRIx64 "..0x%016" PRIx64
            " crosses the 1 TiB requester window\n",
            host_addr, host_last_addr);
    exit(EXIT_FAILURE);
  }

  rn_dev->winSize->win_size_msb = (uint32_t) ((win_size & 0xffffffff00000000) >> 32);
  rn_dev->winSize->win_size_lsb  = (uint32_t) (win_size & 0x00000000ffffffff);

  // Disable every entry before updating entry 0.  ERNIC emits host addresses
  // masked to 40 bits, so AXI base 0 maps to the PCIe DMA-address region
  // containing the pool (IOVA, not CPU physical address, with an IOMMU).
  for(i=0; i<RN_AXI_BR_TABLE_ENTRIES; i++) {
    entry_addr = RN_AXI_BR_TABLE_BASE + i * RN_AXI_BR_ENTRY_SIZE;
    write32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_CTRL_OFFSET, 0);
  }

  entry_addr = RN_AXI_BR_TABLE_BASE;
  write32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_AXI_BASE_LO_OFFSET, 0);
  write32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_AXI_BASE_HI_OFFSET, 0);
  write32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_PCIE_BASE_LO_OFFSET,
               (uint32_t) pcie_base);
  write32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_PCIE_BASE_HI_OFFSET,
               (uint32_t) (pcie_base >> 32));
  write32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_MASK_LO_OFFSET,
               (uint32_t) window_mask);
  write32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_MASK_HI_OFFSET,
               (uint32_t) (window_mask >> 32));
  __sync_synchronize();
  write32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_CTRL_OFFSET, 1);

  if((read32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_CTRL_OFFSET) != 1) ||
     (read32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_AXI_BASE_LO_OFFSET) != 0) ||
     (read32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_AXI_BASE_HI_OFFSET) != 0) ||
     (read32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_PCIE_BASE_LO_OFFSET) != (uint32_t) pcie_base) ||
     (read32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_PCIE_BASE_HI_OFFSET) != (uint32_t) (pcie_base >> 32)) ||
     (read32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_MASK_LO_OFFSET) != (uint32_t) window_mask) ||
     (read32_data(rn_dev->bar0_ctl, entry_addr + RN_AXI_BR_MASK_HI_OFFSET) != (uint32_t) (window_mask >> 32))) {
    fprintf(stderr, "Error: BAR0 AXI requester translation-table readback failed\n");
    exit(EXIT_FAILURE);
  }

  fprintf(stderr,
          "Info: BAR0 requester window AXI 0x0000000000000000..0x%016" PRIx64
          " -> PCIe base 0x%016" PRIx64 "\n",
          win_size, pcie_base);
}

struct rdma_buff_t* allocate_rdma_buffer(struct rn_dev_t* rn_dev, uint64_t buf_size, char* buf_location) {
  struct rdma_buff_t* rdma_buffer;
  rdma_buffer = (struct rdma_buff_t*) malloc(sizeof(struct rdma_buff_t));
  if(rdma_buffer == NULL) {
    fprintf(stderr, "Error: failed to create rdma_buffer\n");
    exit(EXIT_FAILURE);
  }

  if(!strcmp(buf_location, HOST_MEM)) {
    // Allocate the buffer in the host memory
    // Check the buffer_offset and buf_size whether it meets 4KB alignment or not
    if (buf_size <= HARDWARE_PAGE_SIZE) {
      if (((rn_dev->buffer_offset & HARDWARE_PAGE_SIZE_ADDRESS_MASK) + buf_size) > HARDWARE_PAGE_SIZE) {
        rn_dev->buffer_offset =  (rn_dev->buffer_offset + HARDWARE_PAGE_SIZE) & HARDWARE_PAGE_SIZE_ALIGNMENT_MASK;
      }
    } else {
      // buf_size > HARDWARE_PAGE_SIZE
      if((rn_dev->buffer_offset & HARDWARE_PAGE_SIZE_ADDRESS_MASK) != 0) {
        // buffer_offset is not aligned with HARDWARE_PAGE_SIZE
        rn_dev->buffer_offset =  (rn_dev->buffer_offset + HARDWARE_PAGE_SIZE) & HARDWARE_PAGE_SIZE_ALIGNMENT_MASK;
      }
    }
    if((rn_dev->buffer_offset > rn_dev->hugepage_map_size) ||
       (buf_size > rn_dev->hugepage_map_size - rn_dev->buffer_offset)) {
      fprintf(stderr,
              "Error: host RDMA buffer pool exhausted (offset=0x%" PRIx64
              ", request=0x%" PRIx64 ", pool=0x%" PRIx64 ")\n",
              rn_dev->buffer_offset, buf_size, rn_dev->hugepage_map_size);
      exit(EXIT_FAILURE);
    }
    rdma_buffer->buffer = (void*)((uint64_t) rn_dev->base_buf->buffer + rn_dev->buffer_offset);
    rn_dev->buffer_offset += buf_size;
    rdma_buffer->buf_size = buf_size;

    // A coherent allocation is contiguous in DMA address space even when
    // its underlying CPU physical pages are not contiguous.
    if(rn_dev->dma_pool.cpu != NULL) {
      rdma_buffer->dma_addr = rn_dev->base_buf->dma_addr +
                              (rn_dev->buffer_offset - buf_size);
    } else {
      rdma_buffer->dma_addr = get_buffer_paddr(rdma_buffer->buffer);
    }
    Debug("Info: allocated host buffer vir addr = %p, DMA addr = %lx, rn_dev->buffer_offset = 0x%lx\n", rdma_buffer->buffer, rdma_buffer->dma_addr, rn_dev->buffer_offset);
    Debug("Info: allocate_rdma_buffer - successfully allocated rdma host buffer\n");
  } else {
    if (!strcmp(buf_location, DEVICE_MEM)) {
      // Allocate the buffer in the device memory
      // TODO: We need to implement user-space device memory management function
      // Check the dev_buffer_offset and buf_size whether it meets 4KB alignment or not
      if (buf_size <= HARDWARE_PAGE_SIZE) {
        if (((rn_dev->dev_buffer_offset & HARDWARE_PAGE_SIZE_ADDRESS_MASK) + buf_size) > HARDWARE_PAGE_SIZE) {
          rn_dev->dev_buffer_offset =  (rn_dev->dev_buffer_offset + HARDWARE_PAGE_SIZE) & HARDWARE_PAGE_SIZE_ALIGNMENT_MASK;
        }
      } else {
        // buf_size > HARDWARE_PAGE_SIZE
        if((rn_dev->dev_buffer_offset & HARDWARE_PAGE_SIZE_ADDRESS_MASK) != 0) {
          // dev_buffer_offset is not aligned with HARDWARE_PAGE_SIZE
          rn_dev->dev_buffer_offset =  (rn_dev->dev_buffer_offset + HARDWARE_PAGE_SIZE) & HARDWARE_PAGE_SIZE_ALIGNMENT_MASK;
        }
      }
      rdma_buffer->buffer = (void*)(rn_dev->dev_buffer_offset | DEVICE_MEM_OFFSET);
      rdma_buffer->dma_addr = (uint64_t) (rn_dev->dev_buffer_offset | DEVICE_MEM_OFFSET);
      rn_dev->dev_buffer_offset += buf_size;
      rdma_buffer->buf_size = buf_size;

      // TODO: We need to put assert here later to make sure we won't exceed device memory.
      Debug("Info: allocated device buffer physical addr = %lx, rn_dev->dev_buffer_offset = 0x%lx\n", rdma_buffer->dma_addr, rn_dev->dev_buffer_offset);
      assert(rn_dev->dev_buffer_offset <= (uint64_t) DEVICE_MEM_SIZE);
    Debug("Info: allocate_rdma_buffer - successfully allocated rdma device buffer\n");
    } else {
      fprintf(stderr, "Error: please provide correct buffer location: [host_mem | dev_mem]\n");
      exit(EXIT_FAILURE);
    }
  }

  return rdma_buffer;
}

static struct rn_dev_t* create_rn_dev_internal(char* pcie_resource, int* pcie_resource_fd, uint32_t num_hugepages_request, uint32_t num_qp, const char* dma_device) {
  int scr;
  void* axil_scr_base;
  void* bar0_base;
  uint32_t phy_addr_msb;
  uint32_t phy_addr_lsb;
  char bar0_resource[PATH_MAX];

  struct rn_dev_t* rn_dev = NULL;
  struct win_size_t* winSize = NULL;

  rn_dev = (struct rn_dev_t* ) calloc(1, sizeof(struct rn_dev_t));
  winSize = (struct win_size_t* ) calloc(1, sizeof(struct win_size_t));

  if((rn_dev == NULL) || (winSize == NULL)) {
    fprintf(stderr, "Error: failed to allocate RecoNIC device state\n");
    exit(EXIT_FAILURE);
  }

  if((pcie_resource == NULL) || (pcie_resource_fd == NULL)) {
    fprintf(stderr, "Error: resource2 path is required\n");
    exit(EXIT_FAILURE);
  }

  if((num_qp == 0) || (num_qp > UINT8_MAX)) {
    fprintf(stderr, "Error: num_qp must be in the range 1..%u\n", UINT8_MAX);
    exit(EXIT_FAILURE);
  }

  if(num_hugepages_request == 0) {
    fprintf(stderr, "Error: at least one 2 MiB hugepage is required\n");
    exit(EXIT_FAILURE);
  }

  if(((uint64_t) num_hugepages_request << HUGE_PAGE_SHIFT) > UINT32_MAX) {
    fprintf(stderr, "Error: hugepage pool is larger than the current API supports\n");
    exit(EXIT_FAILURE);
  }

  rn_dev->axil_map_size = RN_SCR_MAP_SIZE;
  rn_dev->bar0_map_size = RN_BAR0_MAP_SIZE;
  rn_dev->bar0_fd = -1;
  rn_dev->dma_pool.fd = -1;
  rn_dev->hugepage_map_size = (uint64_t) num_hugepages_request << HUGE_PAGE_SHIFT;
  rn_dev->rdma_dev = NULL;
  rn_dev->base_buf = NULL;
  rn_dev->winSize = winSize;

  if((scr = open(pcie_resource, O_RDWR | O_SYNC)) == -1) {
    fprintf(stderr, "Error can't open %s file for the PCIe resource2!\n", pcie_resource);
    exit(EXIT_FAILURE);
  }

  *pcie_resource_fd = scr;

  Debug("Info: scr(=%d)) file open successfully\n", scr);

  axil_scr_base = mmap(NULL, RN_SCR_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, scr, 0);

  if (axil_scr_base == MAP_FAILED) {
    fprintf(stderr, "Error: axil_scr_base mmap failed\n");
    close(scr);
    exit(EXIT_FAILURE);
  }

  rn_dev->axil_ctl = (uint32_t* ) axil_scr_base;
  rn_dev->num_qp = num_qp;

  if(get_bar0_resource_path(pcie_resource, bar0_resource,
                            sizeof(bar0_resource)) != 0) {
    fprintf(stderr, "Error: expected a sysfs path ending in /resource2: %s\n",
            pcie_resource);
    exit(EXIT_FAILURE);
  }

  rn_dev->bar0_fd = open(bar0_resource, O_RDWR | O_SYNC);
  if(rn_dev->bar0_fd == -1) {
    fprintf(stderr, "Error: can't open BAR0 resource %s: %s\n",
            bar0_resource, strerror(errno));
    exit(EXIT_FAILURE);
  }

  bar0_base = mmap(NULL, rn_dev->bar0_map_size, PROT_READ | PROT_WRITE,
                   MAP_SHARED, rn_dev->bar0_fd, 0);
  if(bar0_base == MAP_FAILED) {
    fprintf(stderr, "Error: BAR0 mmap failed for %s: %s\n",
            bar0_resource, strerror(errno));
    exit(EXIT_FAILURE);
  }
  rn_dev->bar0_ctl = (uint32_t* ) bar0_base;

  // Allocate the host pool before enabling ERNIC or posting any WQE.
  rn_dev->base_buf = (struct rdma_buff_t*) malloc(sizeof(struct rdma_buff_t));
  if(rn_dev->base_buf == NULL) {
    fprintf(stderr, "Error: failed to create rn_dev->base_buf\n");
    exit(EXIT_FAILURE);
  }

  if(dma_device != NULL) {
    if(read32_data(rn_dev->axil_ctl, RN_RDMA_GCSR_XRNICCONF) & 1U) {
      fprintf(stderr, "Error: ERNIC is already enabled; stop other users and "
              "reset FPGA/ERNIC before replacing its DMA mappings\n");
      exit(EXIT_FAILURE);
    }
    if(rn_dma_pool_open(&rn_dev->dma_pool, dma_device, pcie_resource,
                        rn_dev->hugepage_map_size) != 0) {
      fprintf(stderr, "Error: coherent DMA pool on %s: %s; check driver ABI "
              "and PCI device ownership. No physical-address fallback.\n",
              dma_device, strerror(errno));
      exit(EXIT_FAILURE);
    }
    rn_dev->hugepage_map_size = rn_dev->dma_pool.size;
    rn_dev->base_buf->buffer = rn_dev->dma_pool.cpu;
    rn_dev->base_buf->buf_size = (uint32_t)rn_dev->dma_pool.size;
    rn_dev->base_buf->dma_addr = rn_dev->dma_pool.dma_addr;
    fprintf(stderr, "Info: coherent DMA pool %s: CPU VA=%p, DMA/IOVA=0x%016"
            PRIx64 ", size=%zu (no HugeTLB/pagemap)\n", dma_device,
            rn_dev->dma_pool.cpu, rn_dev->dma_pool.dma_addr,
            rn_dev->dma_pool.size);
  } else {
    rn_dev->base_buf->buffer = mmap(NULL, rn_dev->hugepage_map_size,
                                  PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS |
                                  MAP_HUGETLB, -1, 0);

    if(rn_dev->base_buf->buffer == MAP_FAILED) {
      fprintf(stderr, "Error: failed to allocate %u 2 MiB hugepage(s): %s\n",
            num_hugepages_request, strerror(errno));
      exit(EXIT_FAILURE);
    }
    rn_dev->base_buf->buf_size = (uint32_t) rn_dev->hugepage_map_size;

    // Lock the buffer in physical memory
    if(mlock(rn_dev->base_buf->buffer, rn_dev->hugepage_map_size) == -1) {
      fprintf(stderr, "Error: failed to lock page in memory\n");
      exit(EXIT_FAILURE);
    }

    rn_dev->base_buf->dma_addr = get_buffer_paddr(rn_dev->base_buf->buffer);
    fprintf(stderr, "Info: pre-allocated hugepage buffer vir addr = %p, physical addr = 0x%lx\n", rn_dev->base_buf->buffer, rn_dev->base_buf->dma_addr);

    // The legacy userspace API has no scatter/gather page table. Refuse a
    // virtually contiguous but physically fragmented hugepage pool instead
    // of issuing DMA to unrelated physical memory.
    for(uint64_t offset = UINT64_C(1) << HUGE_PAGE_SHIFT;
      offset < rn_dev->hugepage_map_size;
      offset += UINT64_C(1) << HUGE_PAGE_SHIFT) {
      uint64_t page_addr = get_buffer_paddr(
      (uint8_t*) rn_dev->base_buf->buffer + offset);
      if(page_addr != rn_dev->base_buf->dma_addr + offset) {
        fprintf(stderr,
              "Error: hugepage pool is not physically contiguous at offset "
              "0x%" PRIx64 " (expected 0x%" PRIx64 ", got 0x%" PRIx64 ")\n",
              offset, rn_dev->base_buf->dma_addr + offset, page_addr);
        exit(EXIT_FAILURE);
      }
    }

  } // Legacy physically addressed HugeTLB backend.

  phy_addr_msb = (uint32_t) ((rn_dev->base_buf->dma_addr & 0xffffffff00000000) >> 32);
  phy_addr_lsb = (uint32_t) ((rn_dev->base_buf->dma_addr & 0x00000000ffffffff));

  // Configure the ZDMA AXI-MM requester bridge through BAR0.
  config_rn_dev_axib_bdf(rn_dev, phy_addr_msb, phy_addr_lsb);

  rn_dev->buffer_offset = (uint64_t) 0;
  rn_dev->dev_buffer_offset = (uint64_t) 0;

  return rn_dev;
}

struct rn_dev_t* create_rn_dev(char* pcie_resource, int* pcie_resource_fd,
                              uint32_t num_hugepages_request, uint32_t num_qp) {
  return create_rn_dev_internal(pcie_resource, pcie_resource_fd,
                                num_hugepages_request, num_qp, NULL);
}

struct rn_dev_t* create_rn_dev_dma(char* pcie_resource, int* pcie_resource_fd,
                                  uint32_t pool_units, uint32_t num_qp,
                                  const char* dma_device) {
  if(!dma_device || !*dma_device) {
    fprintf(stderr, "Error: a coherent DMA character device is required\n");
    exit(EXIT_FAILURE);
  }
  return create_rn_dev_internal(pcie_resource, pcie_resource_fd,
                                pool_units, num_qp, dma_device);
}
