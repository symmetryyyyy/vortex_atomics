#include <vx_spawn.h>
#include <vx_intrinsics.h>
#include "common.h"

#define TILE_MAX 64

void kernel_body(kernel_arg_t* __UNIFORM__ arg) {
  __shared__ uint32_t smem[2][TILE_MAX];
  __shared__ vx_pipeline_desc_t pipe_desc;
  __shared__ vx_tensor_desc_t tma_desc[2];

  if (threadIdx.x == 0) {
    pipe_desc.base_addr = (uint64_t)smem;
    pipe_desc.stage_bytes = arg->tile_size * sizeof(uint32_t);
    pipe_desc.num_stages = 2;
    pipe_desc.barrier_base = 0;
    pipe_desc.expected_arrivals = 1;
    vx_pipeline_init(0, &pipe_desc);

    for (int i = 0; i < 2; ++i) {
      tma_desc[i].base = 0;
      tma_desc[i].dims[0] = 0;
      tma_desc[i].strides[0] = sizeof(uint32_t);
      tma_desc[i].ndim = 1;
      tma_desc[i].elem_size = sizeof(uint32_t);
    }
  }

  uint32_t* src_ptr = reinterpret_cast<uint32_t*>(arg->src_addr);
  uint32_t* dst_ptr = reinterpret_cast<uint32_t*>(arg->dst_addr);
  uint32_t tile_size = arg->tile_size;

  for (uint32_t offset = 0; offset < arg->count; offset += tile_size) {
    uint32_t stage = (offset / tile_size) & 1;
    uint32_t remaining = arg->count - offset;
    uint32_t tile_elems = remaining > tile_size ? tile_size : remaining;

    if (threadIdx.x == 0) {
      tma_desc[stage].base = (uint64_t)(src_ptr + offset);
      tma_desc[stage].dims[0] = tile_elems;
      vx_tma_load_async(&tma_desc[stage], 0, stage);
    }

    vx_pipeline_consumer_wait(0, stage);

    if (threadIdx.x == 0) {
      for (uint32_t i = 0; i < tile_elems; ++i) {
        dst_ptr[offset + i] = smem[stage][i];
      }
      vx_pipeline_consumer_release(0, stage);
    }
  }
}

int main() {
  kernel_arg_t* arg = (kernel_arg_t*)csr_read(VX_CSR_MSCRATCH);
  return vx_spawn_threads(1, &arg->count, nullptr, (vx_kernel_func_cb)kernel_body, arg);
}
