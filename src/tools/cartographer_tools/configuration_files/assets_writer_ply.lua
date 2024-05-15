-- Copyright 2016 The Cartographer Authors
-- Copyright 2024 Ekumen, Inc.
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

options = {
  tracking_frame = "base_link",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 60.,
    },
    {
      action = "dump_num_points",
    },
    {
      action = "voxel_filter_and_remove_moving_objects",
      voxel_size = 0.05,
    },
    {
      action = "intensity_to_color",
      min_intensity = 0.,
      max_intensity = 4095.,
    },
    {
      action = "write_ply",
      filename = "points.ply",
    },
  }
}

return options
