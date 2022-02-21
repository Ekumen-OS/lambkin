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

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/system/filesystem.h>
#include <rosbag/bag.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rawlog2bag/groundtruth_stream.h"
#include "rawlog2bag/junction_stream.h"
#include "rawlog2bag/rawlog_stream.h"
#include "rawlog2bag/sequential_stream.h"

namespace rawlog2bag
{
namespace
{
std::string rtrim(const std::string &str, const std::string &end)
{
  if (str.size() >= end.size()) {
    if (std::equal(end.rbegin(), end.rend(), str.rbegin())) {
      return str.substr(0, str.size() - end.size());
    }
  }
  return str;
}
}  // namespace

int main(int argc, char **argv)
{
  TCLAP::CmdLine parser(
      "Convert Rawlog files into ROS 1 bagfiles", ' ', "0.1.0");
  TCLAP::ValueArg<std::string> output_bag_flag(
      "o", "output-bag",
      "Path to output bag file. It defaults to rawlog file path,"
      " minus a .rawlog extension, plus a .bag extension.",
      false, "", "path");
  TCLAP::MultiArg<std::string> groundtruth_files_flag(
      "g", "groundtruth-file", "Path to groundtruth text file.",
      false, "path");
  TCLAP::UnlabeledValueArg<std::string> input_rawlog_arg(
      "input_rawlog", "Path to input rawlog file.", true, "", "path");
  parser.add(output_bag_flag);
  parser.add(groundtruth_files_flag);
  parser.add(input_rawlog_arg);
  if (!parser.parse(argc, argv)) {
    return 1;
  }

  const std::string rawlog_path = input_rawlog_arg.getValue();
  std::vector<std::unique_ptr<SequentialInputStream>> streams;
  streams.emplace_back(new RawlogInputStream(rawlog_path));
  if (groundtruth_files_flag.isSet()) {
    for (const std::string &path : groundtruth_files_flag.getValue()) {
      streams.emplace_back(new GroundtruthInputStream(
          path, mrpt::system::extractFileName(path)));
    }
  }
  const std::string bag_path =
      output_bag_flag.isSet() ?
          output_bag_flag.getValue() :
          rtrim(rawlog_path, ".rawlog") + ".bag";
  rosbag::Bag bag(bag_path, rosbag::bagmode::Write);
  bag.setCompression(rosbag::compression::BZ2);

  JunctionInputStream stream{std::move(streams)};
  while (!stream.eof()) {
    auto record = stream.read();
    if (record) {
      record->writeback(&bag);
    }
  }
  return 0;
}

}  // namespace rawlog2bag

int main(int argc, char **argv)
{
  return rawlog2bag::main(argc, argv);
}
