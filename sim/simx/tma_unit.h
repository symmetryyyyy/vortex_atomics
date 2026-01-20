// Copyright © 2019-2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>
#include <deque>

namespace vortex {

class Emulator;

struct tensor_desc_t {
  uint64_t base;
  int32_t dims[5];
  int32_t strides[5];
  int32_t ndim;
  int32_t elem_size;
};

struct tma_store_desc_t {
  uint64_t src;
  uint32_t bytes;
  uint32_t group_id;
  tensor_desc_t dst;
};

class TmaUnit {
public:
  explicit TmaUnit(Emulator* emulator);

  void reset();

  void enqueue_load(const tensor_desc_t& desc,
                    uint64_t dst_addr,
                    uint32_t bar_id,
                    uint32_t pipeline_id,
                    uint32_t stage_id);

  void enqueue_store(const tma_store_desc_t& desc);

  void tick();

private:
  struct LoadRequest {
    tensor_desc_t desc;
    uint64_t dst_addr;
    uint32_t bytes;
    uint32_t bar_id;
    uint32_t pipeline_id;
    uint32_t stage_id;
    uint32_t offset;
  };

  struct StoreRequest {
    tma_store_desc_t desc;
    uint32_t offset;
  };

  Emulator* emulator_;
  std::deque<LoadRequest> load_queue_;
  std::deque<StoreRequest> store_queue_;
};

}
