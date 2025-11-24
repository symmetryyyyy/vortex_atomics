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

  // Barrier IDs
  // We need two barriers: one for "Load Done" and one for "Compute Done"
  // Since we are double buffering, we can conceptually think of them as:
  // bar_load: signals that a buffer is filled and ready for compute
  // bar_compute: signals that a buffer is consumed and ready for refill
  // To avoid race conditions, we use separate barrier IDs.
  // Assuming we have enough hardware barriers, we map them based on local group ID.
  // bar_load: used to wait for data loading
  // bar_compute: used to wait for computation completion before overwriting
  int bar_load = __local_group_id * 2;
  int bar_compute = __local_group_id * 2 + 1;

  // Prologue: Load the first tile (Buffer 0)
  if (0 < size) {
      local_A0[l_row * tile_size + l_col] = A_ptr[g_row * size + (0 + l_col)];
      local_B0[l_row * tile_size + l_col] = B_ptr[(0 + l_row) * size + g_col];
  }
  // Wait for Buffer 0 to be loaded. 
  // We use a full barrier here because it's the first step.
  vx_barrier(bar_load, num_warps);

  // Main loop with double buffering
  for (uint32_t k = 0; k < size; k += tile_size) {
    int buf_cur = (k / tile_size) % 2;       
    int buf_next = 1 - buf_cur;             
    
    auto cur_A = (buf_cur == 0) ? local_A0 : local_A1;
    auto cur_B = (buf_cur == 0) ? local_B0 : local_B1;
    auto next_A = (buf_next == 0) ? local_A0 : local_A1;
    auto next_B = (buf_next == 0) ? local_B0 : local_B1;

    uint32_t next_k = k + tile_size;

    // 1. Async Load Next Tile
    if (next_k < size) {
      // Wait for the previous computation on buf_next to finish before overwriting it.
      // For the first iteration (k=0), buf_next (Buffer 1) is empty, so no wait needed.
      if (k > 0) {
        vx_barrier_wait(bar_compute, num_warps);
      }

      // Load data into buf_next
      next_A[l_row * tile_size + l_col] = A_ptr[g_row * size + (next_k + l_col)];
      next_B[l_row * tile_size + l_col] = B_ptr[(next_k + l_row) * size + g_col];
      
      // Signal that loading for buf_next is done (Arrive)
      vx_barrier_arrive(bar_load, num_warps);
    }

    // 2. Compute Current Tile
    // At this point, buf_cur is guaranteed to be ready (from Prologue or previous loop's Wait)
    for (uint32_t j = 0; j < tile_size; ++j) {
      sum += cur_A[l_row * tile_size + j] * cur_B[j * tile_size + l_col];
    }

    // Signal that computation on buf_cur is done (Arrive)
    // This tells the loader it's safe to overwrite buf_cur in the next-next iteration
    vx_barrier_arrive(bar_compute, num_warps);

    // 3. Wait for Next Tile
    // Before moving to the next iteration, we must ensure buf_next is fully loaded.
    if (next_k < size) {
      vx_barrier_wait(bar_load, num_warps);
    }
  }

  // Store the computed sum into the result matrix C
  C_ptr[g_row * size + g_col] = sum;
}

int main() {
  auto arg = (kernel_arg_t*)csr_read(VX_CSR_MSCRATCH);
	return vx_spawn_threads(2, arg->grid_dim, arg->block_dim, (vx_kernel_func_cb)kernel_body, arg);
}
