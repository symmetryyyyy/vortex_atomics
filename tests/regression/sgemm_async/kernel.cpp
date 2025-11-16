#include <vx_spawn.h>
#include "common.h"

void kernel_body(kernel_arg_t *arg) {
	// Setup buffer arguments
  auto A_ptr = reinterpret_cast<TYPE*>(arg->A_addr);
  auto B_ptr = reinterpret_cast<TYPE*>(arg->B_addr);
  auto C_ptr = reinterpret_cast<TYPE*>(arg->C_addr);

  // Allocate local memory for double buffering: 2 buffers for A, 2 for B
	auto local_ptr = __local_mem(4 * blockDim.x * blockDim.y * sizeof(TYPE));
  auto local_A0 = (TYPE*)local_ptr;
  auto local_B0 = local_A0 + blockDim.x * blockDim.y;
  auto local_A1 = local_B0 + blockDim.x * blockDim.y;
  auto local_B1 = local_A1 + blockDim.x * blockDim.y;

  auto size = arg->size;
  auto tile_size = arg->tile_size;

  // Determine global row and column indices
  auto g_row = blockIdx.x * blockDim.x + threadIdx.x;
  auto g_col = blockIdx.y * blockDim.y + threadIdx.y;

  // Determine local row and column indices
  auto l_row = threadIdx.x;
  auto l_col = threadIdx.y;

  TYPE sum(0);

  // 初始化：加载第一个tile到buffer 0并arrive
  local_A0[l_row * tile_size + l_col] = A_ptr[g_row * size + l_col];
  local_B0[l_row * tile_size + l_col] = B_ptr[l_row * size + g_col];
  __syncthreads_arrive();

  // Main loop with double buffering
  for (uint32_t k = 0; k < size; k += tile_size) {
    int buf_cur = (k / tile_size) % 2;  // 当前计算用的buffer
    int buf_next = 1 - buf_cur;          // 下一次加载用的buffer
    
    auto cur_A = (buf_cur == 0) ? local_A0 : local_A1;
    auto cur_B = (buf_cur == 0) ? local_B0 : local_B1;
    auto next_A = (buf_next == 0) ? local_A0 : local_A1;
    auto next_B = (buf_next == 0) ? local_B0 : local_B1;

    // Stage 1: 异步加载下一个tile到next buffer（如果还有）
    if (k + tile_size < size) {
      next_A[l_row * tile_size + l_col] = A_ptr[g_row * size + (k + tile_size + l_col)];
      next_B[l_row * tile_size + l_col] = B_ptr[(k + tile_size + l_row) * size + g_col];
    }

    // Stage 2: 等待当前tile准备好
    __syncthreads_wait();

    // Stage 3: 计算当前tile（使用cur buffer）
    for (uint32_t j = 0; j < tile_size; ++j) {
      sum += cur_A[l_row * tile_size + j] * cur_B[j * tile_size + l_col];
    }

    // Stage 4: 标记下一个tile加载完成（如果还有）
    if (k + tile_size < size) {
      __syncthreads_arrive();
    }
  }

  // Store the computed sum into the result matrix C
  C_ptr[g_row * size + g_col] = sum;
}

int main() {
  auto arg = (kernel_arg_t*)csr_read(VX_CSR_MSCRATCH);
	return vx_spawn_threads(2, arg->grid_dim, arg->block_dim, (vx_kernel_func_cb)kernel_body, arg);
}
