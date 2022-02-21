// Copyright 2022 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "rawlog2bag/junction_stream.h"

namespace rawlog2bag
{
JunctionInputStream::JunctionInputStream(
    std::vector<std::unique_ptr<SequentialInputStream>> streams)
{
  for (auto &stream : streams) {
    while (!stream->eof()) {
      auto record = stream->read();
      if (record) {
        queue_.push_back({std::move(record), std::move(stream)});
        std::push_heap(queue_.begin(), queue_.end());
        break;
      }
    }
  }
}

bool JunctionInputStream::eof() const
{
  return queue_.empty();
}

std::unique_ptr<Record> JunctionInputStream::read()
{
  if (queue_.empty()) {
    return nullptr;
  }
  std::pop_heap(queue_.begin(), queue_.end());
  auto head = std::move(queue_.back());
  queue_.pop_back();

  auto record = std::move(head.record);
  while (!head.stream->eof()) {
    head.record = head.stream->read();
    if (head.record) {
      queue_.push_back(std::move(head));
      std::push_heap(queue_.begin(), queue_.end());
      break;
    }
  }
  return record;
}

}  // namespace rawlog2bag
