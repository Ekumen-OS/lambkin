#include <chrono>
#include <fstream>
#include <iomanip>
#include <string>

#include <cartographer/common/time.h>
#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>

#include <gflags/gflags.h>

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to extract a trajectory from.");
DEFINE_string(trajectory_filestem, "trajectory", "Stem of the output file.");

namespace cartographer_tools {
namespace {

double ToUnix(const cartographer::common::Time& time) {
  constexpr double kUnixEpochOrigin = static_cast<double>(
      cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds);
  const double total_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          time.time_since_epoch()).count();
  return total_seconds - kUnixEpochOrigin;
}

int Run(const std::string& pbstream_filename, const std::string& trajectory_filestem) {
  cartographer::io::ProtoStreamReader reader(pbstream_filename);
  cartographer::io::ProtoStreamDeserializer deserializer(&reader);
  const auto& pose_graph = deserializer.pose_graph();
  for (int i = 0; i < pose_graph.trajectory_size(); ++i) {
    const std::string trajectory_filename =
        pose_graph.trajectory_size() > 1 ?
        trajectory_filestem + "." + std::to_string(i) + ".tum" :
        trajectory_filestem + ".tum";
    std::ofstream trajectory_file{trajectory_filename};
    trajectory_file << std::fixed << std::setprecision(9);
    for (const auto& node : pose_graph.trajectory(i).node()) {
      trajectory_file
          << ToUnix(cartographer::common::FromUniversal(node.timestamp())) << " "
          << node.pose().translation().x() << " "
          << node.pose().translation().y() << " "
          << node.pose().translation().z() << " "
          << node.pose().rotation().x() << " "
          << node.pose().rotation().y() << " "
          << node.pose().rotation().z() << " "
          << node.pose().rotation().w() << std::endl;
    }
  }
  return 0;
}

}  // namespace
}  // namespace cartographer_tools

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  return ::cartographer_tools::Run(FLAGS_pbstream_filename, FLAGS_trajectory_filestem);
}
