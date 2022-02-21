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

#ifndef RAWLOG2BAG_JUNCTION_STREAM_H
#define RAWLOG2BAG_JUNCTION_STREAM_H

#include <memory>
#include <vector>

#include "rawlog2bag/sequential_stream.h"

namespace rawlog2bag
{
class JunctionInputStream : public SequentialInputStream
{
 public:
  explicit JunctionInputStream(
      std::vector<std::unique_ptr<SequentialInputStream>> streams);

  bool eof() const override;

  std::unique_ptr<Record> read() override;

 private:
  struct SequentialInputStreamHead {
    std::unique_ptr<Record> record;
    std::unique_ptr<SequentialInputStream> stream;

    bool operator<(const SequentialInputStreamHead& other) const
    {
      return this->record->timestamp() < other.record->timestamp();
    }
  };
  std::vector<SequentialInputStreamHead> queue_;
};

}  // namespace rawlog2bag

#endif  // RAWLOG2BAG_JUNCTION_STREAM_H
