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

#ifndef RAWLOG2BAG_RAWLOG_STREAM_H
#define RAWLOG2BAG_RAWLOG_STREAM_H

#include <mrpt/obs/CRawlog.h>

#include <memory>
#include <string>

#include "rawlog2bag/sequential_stream.h"

namespace rawlog2bag
{
class RawlogInputStream : public SequentialInputStream
{
 public:
  explicit RawlogInputStream(
      const std::string& path,
      const std::string& base_frame = "base_link",
      bool ignore_unknown = true);

  bool eof() const override;

  std::unique_ptr<Record> read() override;

 private:
  const std::string base_frame_;
  const bool ignore_unknown_;
  mrpt::obs::CRawlog rawlog_{};
  size_t index_{0u};
};

}  // namespace rawlog2bag

#endif  // RAWLOG2BAG_RAWLOG_STREAM_H
