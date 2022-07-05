#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#ifdef ANDROID_10
#include <ion.h>
#include <linux/dma-buf.h>
#elif
#include <linux/ion.h>
#endif

#include <CL/cl_ext.h>
#include <msm_ion.h>

#include "logger/logger.h"
#include "visionbuf.h"

// keep trying if x gets interrupted by a signal
#define HANDLE_EINTR(x)                                       \
  ({                                                          \
    decltype(x) ret;                                          \
    int try_cnt = 0;                                          \
    do {                                                      \
      ret = (x);                                              \
    } while (ret == -1 && errno == EINTR && try_cnt++ < 100); \
    ret;                                                      \
  })

// just hard-code these for convenience
// size_t device_page_size = 0;
// clGetDeviceInfo(device_id, CL_DEVICE_PAGE_SIZE_QCOM,
//                 sizeof(device_page_size), &device_page_size,
//                 NULL);

// size_t padding_cl = 0;
// clGetDeviceInfo(device_id, CL_DEVICE_EXT_MEM_PADDING_IN_BYTES_QCOM,
//                 sizeof(padding_cl), &padding_cl,
//                 NULL);
#define DEVICE_PAGE_SIZE_CL 4096
#define PADDING_CL 0

static int ion_fd = -1;
static void ion_init() {
  if (ion_fd == -1) {
    ion_fd = open("/dev/ion", O_RDWR | O_NONBLOCK);
  }
}

void VisionBuf::allocate(size_t length) {
  int err;

  ion_init();

  struct ion_allocation_data ion_alloc; // = {0};
  ion_alloc.len = length + PADDING_CL + sizeof(uint64_t);
  ion_alloc.align = 4096;
  ion_alloc.heap_id_mask = 1 <<  ION_IOMMU_HEAP_ID;
  ion_alloc.flags = ION_FLAG_CACHED;

#ifdef ANDROID_10
  int tmp_handle;
  err = HANDLE_EINTR(ion_alloc_fd(ion_fd, ion_alloc.len, ion_alloc.align, ion_alloc.heap_id_mask, ion_alloc.flags, &tmp_handle));
  assert(err == 0);
#elif
  struct ion_fd_data ion_fd_data = {0};
  err = HANDLE_EINTR(ioctl(ion_fd, ION_IOC_ALLOC, &ion_alloc));
  assert(err == 0);
  ion_fd_data.handle = ion_alloc.handle;
  err = HANDLE_EINTR(ioctl(ion_fd, ION_IOC_SHARE, &ion_fd_data));
  assert(err == 0);
#endif

#ifdef ANDROID_10
  void *mmap_addr = mmap(NULL, ion_alloc.len,
                         PROT_READ | PROT_WRITE,
                         MAP_SHARED, tmp_handle, 0);
  assert(mmap_addr != MAP_FAILED);
#elif
  void *mmap_addr = mmap(NULL, ion_alloc.len,
                         PROT_READ | PROT_WRITE,
                         MAP_SHARED, ion_fd_data.fd, 0);
  assert(mmap_addr != MAP_FAILED);
#endif

  memset(mmap_addr, 0, ion_alloc.len);

  this->len = length;
  this->mmap_len = ion_alloc.len;
  this->addr = mmap_addr;
#ifdef ANDROID_10
  this->handle = tmp_handle;
  this->fd = tmp_handle;
#elif
  this->handle = ion_alloc.handle;
  this->fd = ion_fd_data.fd;
#endif
  this->frame_id = (uint64_t*)((uint8_t*)this->addr + this->len + PADDING_CL);
}

void VisionBuf::import() {
  assert(this->fd >= 0);

  ion_init();

#ifdef ANDROID_10
  this->handle = this->fd;
#elif
  // Get handle
  int err;
  struct ion_fd_data fd_data = {0};
  fd_data.fd = this->fd;
  err = HANDLE_EINTR(ioctl(ion_fd, ION_IOC_IMPORT, &fd_data));
  assert(err == 0);
  this->handle = fd_data.handle;
#endif

  this->addr = mmap(NULL, this->mmap_len, PROT_READ | PROT_WRITE, MAP_SHARED, this->fd, 0);
  assert(this->addr != MAP_FAILED);

  this->frame_id = (uint64_t*)((uint8_t*)this->addr + this->len + PADDING_CL);
}

void VisionBuf::init_cl(cl_device_id device_id, cl_context ctx) {
  int err;

  assert(((uintptr_t)this->addr % DEVICE_PAGE_SIZE_CL) == 0);

  cl_mem_ion_host_ptr ion_cl = {0};
  ion_cl.ext_host_ptr.allocation_type = CL_MEM_ION_HOST_PTR_QCOM;
  ion_cl.ext_host_ptr.host_cache_policy = CL_MEM_HOST_UNCACHED_QCOM;
  ion_cl.ion_filedesc = this->fd;
  ion_cl.ion_hostptr = this->addr;

  this->buf_cl = clCreateBuffer(ctx,
                              CL_MEM_USE_HOST_PTR | CL_MEM_EXT_HOST_PTR_QCOM,
                              this->len, &ion_cl, &err);
  if (err) LOGD("clCreateBuffer: %d", err);
  assert(err == 0);
}


int VisionBuf::sync(int dir) {
#ifdef ANDROID_10
  struct dma_buf_sync sync = { 0 };
  sync.flags = (dir == VISIONBUF_SYNC_FROM_DEVICE) ? DMA_BUF_SYNC_READ | DMA_BUF_SYNC_START : DMA_BUF_SYNC_READ | DMA_BUF_SYNC_END;
  return HANDLE_EINTR(ioctl(this->handle, DMA_BUF_IOCTL_SYNC, &sync));
#elif
  struct ion_flush_data flush_data = {0};
  flush_data.handle = this->handle;
  flush_data.vaddr = this->addr;
  flush_data.offset = 0;
  flush_data.length = this->len;

  // ION_IOC_INV_CACHES ~= DMA_FROM_DEVICE
  // ION_IOC_CLEAN_CACHES ~= DMA_TO_DEVICE
  // ION_IOC_CLEAN_INV_CACHES ~= DMA_BIDIRECTIONAL

  struct ion_custom_data custom_data = {0};
  assert(dir == VISIONBUF_SYNC_FROM_DEVICE || dir == VISIONBUF_SYNC_TO_DEVICE);
  custom_data.cmd = (dir == VISIONBUF_SYNC_FROM_DEVICE) ? ION_IOC_INV_CACHES : ION_IOC_CLEAN_CACHES;
  custom_data.arg = (unsigned long)&flush_data;
  return HANDLE_EINTR(ioctl(ion_fd, ION_IOC_CUSTOM, &custom_data));
#endif
}

int VisionBuf::free() {
  int err = 0;

  if (this->buf_cl){
    err = clReleaseMemObject(this->buf_cl);
    if (err != 0) return err;
  }

  err = munmap(this->addr, this->mmap_len);
  if (err != 0) return err;

  err = close(this->fd);
  if (err != 0) return err;

#ifdef ANDROID_10
  return 0;
#elif
  struct ion_handle_data handle_data = {.handle = this->handle};
  return HANDLE_EINTR(ioctl(ion_fd, ION_IOC_FREE, &handle_data));
#endif
}
