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

#include "tma_unit.h"
#include "emulator.h"
#include <algorithm>
#include <array>

namespace vortex {

static uint32_t tensor_desc_total_bytes(const tensor_desc_t& desc) {
  uint64_t total = desc.elem_size;
  for (int i = 0; i < desc.ndim; ++i) {
    total *= static_cast<uint64_t>(desc.dims[i]);
  }
  return static_cast<uint32_t>(total);
}

TmaUnit::TmaUnit(Emulator* emulator)
  : emulator_(emulator) {}

void TmaUnit::reset() {
  load_queue_.clear();
  store_queue_.clear();
}

void TmaUnit::enqueue_load(const tensor_desc_t& desc,
                           uint64_t dst_addr,
                           uint32_t bar_id,
                           uint32_t pipeline_id,
                           uint32_t stage_id) {
  LoadRequest req{};
  req.desc = desc;
  req.dst_addr = dst_addr;
  req.bytes = tensor_desc_total_bytes(desc);
  req.bar_id = bar_id;
  req.pipeline_id = pipeline_id;
  req.stage_id = stage_id;
  req.offset = 0;
  load_queue_.push_back(req);
}

void TmaUnit::enqueue_store(const tma_store_desc_t& desc) {
  StoreRequest req{};
  req.desc = desc;
  req.offset = 0;
  store_queue_.push_back(req);
}

void TmaUnit::tick() {
  if (!load_queue_.empty()) {
    auto& req = load_queue_.front();
    std::array<uint8_t, 16> buffer{};
    uint32_t remaining = req.bytes - req.offset;
    uint32_t chunk = std::min<uint32_t>(16, remaining);
    uint64_t src_addr = req.desc.base + req.offset;
    uint64_t dst_addr = req.dst_addr + req.offset;
    emulator_->dcache_read(buffer.data(), src_addr, chunk);
    emulator_->dcache_write(buffer.data(), dst_addr, chunk);
    req.offset += chunk;
    if (req.offset >= req.bytes) {
      emulator_->mbarrier_complete_tx(req.bar_id, req.bytes);
      emulator_->pipeline_mark_ready(req.pipeline_id, req.stage_id);
      load_queue_.pop_front();
    }
    return;
  }

  if (!store_queue_.empty()) {
    auto& req = store_queue_.front();
    std::array<uint8_t, 16> buffer{};
    uint32_t remaining = req.desc.bytes - req.offset;
    uint32_t chunk = std::min<uint32_t>(16, remaining);
    uint64_t src_addr = req.desc.src + req.offset;
    uint64_t dst_addr = req.desc.dst.base + req.offset;
    emulator_->dcache_read(buffer.data(), src_addr, chunk);
    emulator_->dcache_write(buffer.data(), dst_addr, chunk);
    req.offset += chunk;
    if (req.offset >= req.desc.bytes) {
      emulator_->async_group_complete(req.desc.group_id);
      store_queue_.pop_front();
    }
  }
}

}
