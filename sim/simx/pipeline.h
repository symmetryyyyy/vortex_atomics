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

#include "instr_trace.h"
#include <cstdint>
#include <queue>
#include <vector>

namespace vortex {

class PipelineLatch {
public:
  PipelineLatch() {}
  ~PipelineLatch() {}

  bool empty() const {
    return queue_.empty();
  }

  instr_trace_t* front() {
    return queue_.front();
  }

  void push(instr_trace_t* value) {
    queue_.push(value);
  }

  void pop() {
    queue_.pop();
  }

  void reset() {
    std::queue<instr_trace_t*> empty;
    std::swap(queue_, empty);
  }

protected:
  std::queue<instr_trace_t*> queue_;
};

enum class PipelineStageState {
  Empty,
  BusyCopy,
  Ready
};

struct PipelineDesc {
  uint64_t base_addr;
  uint32_t stage_bytes;
  uint32_t num_stages;
  uint32_t barrier_base;
  uint32_t expected_arrivals;
};

struct PipelineStage {
  uint64_t buffer_addr;
  uint32_t stage_bytes;
  uint32_t barrier_id;
  PipelineStageState state;
  uint32_t last_token;
};

class AsyncPipeline {
public:
  AsyncPipeline();

  void reset();

  void init(const PipelineDesc& desc);

  uint32_t num_stages() const {
    return static_cast<uint32_t>(stages_.size());
  }

  uint32_t next_stage() const {
    return next_stage_;
  }

  PipelineStage& stage(uint32_t index) {
    return stages_.at(index);
  }

  const PipelineStage& stage(uint32_t index) const {
    return stages_.at(index);
  }

  uint32_t acquire_stage();

  void release_stage(uint32_t stage);

  void mark_ready(uint32_t stage);

private:
  std::vector<PipelineStage> stages_;
  uint32_t next_stage_;
};

}
