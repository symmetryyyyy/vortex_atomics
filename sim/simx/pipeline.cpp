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

#include "pipeline.h"

namespace vortex {

AsyncPipeline::AsyncPipeline()
  : next_stage_(0) {}

void AsyncPipeline::reset() {
  stages_.clear();
  next_stage_ = 0;
}

void AsyncPipeline::init(const PipelineDesc& desc) {
  stages_.clear();
  stages_.reserve(desc.num_stages);
  for (uint32_t i = 0; i < desc.num_stages; ++i) {
    PipelineStage stage{};
    stage.buffer_addr = desc.base_addr + static_cast<uint64_t>(i) * desc.stage_bytes;
    stage.stage_bytes = desc.stage_bytes;
    stage.barrier_id = desc.barrier_base + i;
    stage.state = PipelineStageState::Empty;
    stage.last_token = 0;
    stages_.push_back(stage);
  }
  next_stage_ = 0;
}

uint32_t AsyncPipeline::acquire_stage() {
  if (stages_.empty()) {
    return 0xffffffff;
  }
  for (uint32_t i = 0; i < stages_.size(); ++i) {
    uint32_t idx = (next_stage_ + i) % stages_.size();
    if (stages_[idx].state == PipelineStageState::Empty) {
      next_stage_ = (idx + 1) % stages_.size();
      return idx;
    }
  }
  return 0xffffffff;
}

void AsyncPipeline::release_stage(uint32_t stage) {
  stages_.at(stage).state = PipelineStageState::Empty;
}

void AsyncPipeline::mark_ready(uint32_t stage) {
  auto& entry = stages_.at(stage);
  if (entry.state == PipelineStageState::BusyCopy) {
    entry.state = PipelineStageState::Ready;
  }
}

}
