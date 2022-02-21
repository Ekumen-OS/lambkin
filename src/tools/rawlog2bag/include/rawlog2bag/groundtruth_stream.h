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

#ifndef RAWLOG2BAG_GROUNDTRUTH_STREAM_H
#define RAWLOG2BAG_GROUNDTRUTH_STREAM_H

#include <fstream>
#include <memory>
#include <string>

#include "rawlog2bag/sequential_stream.h"

namespace rawlog2bag
{
class GroundtruthInputStream : public SequentialInputStream
{
 public:
  explicit GroundtruthInputStream(
      const std::string& path,
      const std::string& target_frame = "gt",
      const std::string& reference_frame = "world");

  bool eof() const override;

  std::unique_ptr<Record> read() override;

 private:
  std::ifstream file_;
  const std::string target_frame_;
  const std::string reference_frame_;
};

}  // namespace rawlog2bag

#endif  // RAWLOG2BAG_GROUNDTRUTH_STREAM_H_
