// Copyright 2026 TU Delft / SAM XL
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

#pragma once

#include <cstddef>

#include "riskam_msgs/msg/person_detection.hpp"

namespace coresense_people_estimation
{

/// COCO-17 keypoint indices, in the order riskam_msgs documents.
constexpr std::size_t kNumKeypoints = 17;
constexpr std::size_t kLeftShoulder = 5;
constexpr std::size_t kRightShoulder = 6;
constexpr std::size_t kLeftHip = 11;
constexpr std::size_t kRightHip = 12;

/// An undetected keypoint is reported as exactly (0, 0).
inline bool keypointPresent(double x, double y)
{
  return x != 0.0 || y != 0.0;
}

/// The same test against the flat `keypoints` array, which stores each point as
/// an adjacent (x, y) pair.
inline bool keypointPresent(const riskam_msgs::msg::PersonDetection& detection, std::size_t k)
{
  return keypointPresent(detection.keypoints[2 * k], detection.keypoints[2 * k + 1]);
}

/// How many of the 17 keypoints the detector actually placed.
inline std::size_t countValidKeypoints(const riskam_msgs::msg::PersonDetection& detection)
{
  std::size_t n_valid = 0;
  for (std::size_t k = 0; k < kNumKeypoints; ++k)
  {
    if (keypointPresent(detection, k))
    {
      ++n_valid;
    }
  }
  return n_valid;
}

}  // namespace coresense_people_estimation
