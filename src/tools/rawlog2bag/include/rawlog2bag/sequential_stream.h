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

#ifndef RAWLOG2BAG_SEQUENTIAL_STREAM_H
#define RAWLOG2BAG_SEQUENTIAL_STREAM_H

#include <mrpt/system/datetime.h>
#include <rosbag/bag.h>

#include <memory>

namespace rawlog2bag
{
class Record
{
 public:
  virtual mrpt::system::TTimeStamp timestamp() const = 0;

  virtual void writeback(rosbag::Bag* bag) const = 0;
};

class SequentialInputStream
{
 public:
  virtual bool eof() const = 0;

  virtual std::unique_ptr<Record> read() = 0;
};

}  // namespace rawlog2bag

#endif  // RAWLOG2BAG_SEQUENTIAL_STREAM_H
