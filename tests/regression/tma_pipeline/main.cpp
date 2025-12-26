#include <iostream>
#include <unistd.h>
#include <vector>
#include <vortex.h>
#include "common.h"

#define RT_CHECK(_expr)                                         \
   do {                                                         \
     int _ret = _expr;                                          \
     if (0 == _ret)                                             \
       break;                                                   \
     printf("Error: '%s' returned %d!\n", #_expr, (int)_ret);   \
     cleanup();                                                 \
     exit(-1);                                                  \
   } while (false)

const char* kernel_file = "kernel.vxbin";
uint32_t size = 64;
uint32_t tile_size = 32;

vx_device_h device = nullptr;
vx_buffer_h src_buffer = nullptr;
vx_buffer_h dst_buffer = nullptr;
vx_buffer_h krnl_buffer = nullptr;
vx_buffer_h args_buffer = nullptr;
kernel_arg_t kernel_arg = {};

static void show_usage() {
  std::cout << "Vortex TMA pipeline demo." << std::endl;
  std::cout << "Usage: [-k: kernel] [-n words] [-t tile] [-h: help]" << std::endl;
}

static void parse_args(int argc, char **argv) {
  int c;
  while ((c = getopt(argc, argv, "n:k:t:h")) != -1) {
    switch (c) {
    case 'n':
      size = atoi(optarg);
      break;
    case 't':
      tile_size = atoi(optarg);
      break;
    case 'k':
      kernel_file = optarg;
      break;
    case 'h':
      show_usage();
      exit(0);
      break;
    default:
      show_usage();
      exit(-1);
    }
  }
}

void cleanup() {
  if (device) {
    vx_mem_free(src_buffer);
    vx_mem_free(dst_buffer);
    vx_mem_free(krnl_buffer);
    vx_mem_free(args_buffer);
    vx_dev_close(device);
  }
}

int main(int argc, char *argv[]) {
  parse_args(argc, argv);

  std::cout << "open device connection" << std::endl;
  RT_CHECK(vx_dev_open(&device));

  uint32_t buf_size = size * sizeof(uint32_t);

  kernel_arg.count = size;
  kernel_arg.tile_size = tile_size;

  std::cout << "allocate device memory" << std::endl;
  RT_CHECK(vx_mem_alloc(device, buf_size, VX_MEM_READ, &src_buffer));
  RT_CHECK(vx_mem_address(src_buffer, &kernel_arg.src_addr));
  RT_CHECK(vx_mem_alloc(device, buf_size, VX_MEM_WRITE, &dst_buffer));
  RT_CHECK(vx_mem_address(dst_buffer, &kernel_arg.dst_addr));

  std::cout << "dev_src=0x" << std::hex << kernel_arg.src_addr << std::endl;
  std::cout << "dev_dst=0x" << std::hex << kernel_arg.dst_addr << std::endl;

  std::vector<uint32_t> h_src(size);
  std::vector<uint32_t> h_dst(size, 0);
  for (uint32_t i = 0; i < size; ++i) {
    h_src[i] = i + 1;
  }

  std::cout << "upload source buffer" << std::endl;
  RT_CHECK(vx_copy_to_dev(src_buffer, h_src.data(), 0, buf_size));

  std::cout << "Upload kernel binary" << std::endl;
  RT_CHECK(vx_upload_kernel_file(device, kernel_file, &krnl_buffer));

  std::cout << "upload kernel argument" << std::endl;
  RT_CHECK(vx_upload_bytes(device, &kernel_arg, sizeof(kernel_arg_t), &args_buffer));

  std::cout << "start device" << std::endl;
  RT_CHECK(vx_start(device, krnl_buffer, args_buffer));

  std::cout << "wait for completion" << std::endl;
  RT_CHECK(vx_ready_wait(device, VX_MAX_TIMEOUT));

  std::cout << "download destination buffer" << std::endl;
  RT_CHECK(vx_copy_from_dev(h_dst.data(), dst_buffer, 0, buf_size));

  std::cout << "verify result" << std::endl;
  int errors = 0;
  for (uint32_t i = 0; i < size; ++i) {
    if (h_dst[i] != h_src[i]) {
      if (errors < 10) {
        std::cout << "*** error: [" << i << "] expected=" << h_src[i]
                  << " actual=" << h_dst[i] << std::endl;
      }
      ++errors;
    }
  }

  std::cout << "cleanup" << std::endl;
  cleanup();

  if (errors != 0) {
    std::cout << "Found " << std::dec << errors << " errors!" << std::endl;
    std::cout << "FAILED!" << std::endl;
    return 1;
  }

  std::cout << "PASSED!" << std::endl;
  return 0;
}
