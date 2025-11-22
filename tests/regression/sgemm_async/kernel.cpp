#include <vx_spawn.h>
#include <vx_intrinsics.h>
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

  // Calculate number of warps in this block
  uint32_t num_threads = blockDim.x * blockDim.y;
  uint32_t num_warps = (num_threads + vx_num_threads() - 1) / vx_num_threads();

  TYPE sum(0);

  // 
  local_A0[l_row * tile_size + l_col] = A_ptr[g_row * size + l_col];
  local_B0[l_row * tile_size + l_col] = B_ptr[l_row * size + g_col];
  vx_barrier_arrive(0, num_warps);  // barrier 0 for buffer 0

  // Main loop with double buffering
  for (uint32_t k = 0; k < size; k += tile_size) {
    int buf_cur = (k / tile_size) % 2;       
    int buf_next = 1 - buf_cur;             
    int barrier_cur = buf_cur;             
    int barrier_next = buf_next;             
    
    auto cur_A = (buf_cur == 0) ? local_A0 : local_A1;
    auto cur_B = (buf_cur == 0) ? local_B0 : local_B1;
    auto next_A = (buf_next == 0) ? local_A0 : local_A1;
    auto next_B = (buf_next == 0) ? local_B0 : local_B1;

    
    vx_barrier_wait(barrier_cur, num_warps);

   
    if (k + tile_size < size) {
      next_A[l_row * tile_size + l_col] = A_ptr[g_row * size + (k + tile_size + l_col)];
      next_B[l_row * tile_size + l_col] = B_ptr[(k + tile_size + l_row) * size + g_col];
      
      vx_barrier_arrive(barrier_next, num_warps);
    }

    
    for (uint32_t j = 0; j < tile_size; ++j) {
      sum += cur_A[l_row * tile_size + j] * cur_B[j * tile_size + l_col];
    }
  }

  // Store the computed sum into the result matrix C
  C_ptr[g_row * size + g_col] = sum;
}

int main() {
  auto arg = (kernel_arg_t*)csr_read(VX_CSR_MSCRATCH);
	return vx_spawn_threads(2, arg->grid_dim, arg->block_dim, (vx_kernel_func_cb)kernel_body, arg);
}
