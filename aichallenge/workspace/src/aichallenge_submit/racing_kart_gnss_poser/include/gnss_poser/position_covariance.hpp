// Copyright 2026 Tier IV, Inc.
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

#ifndef GNSS_POSER__POSITION_COVARIANCE_HPP_
#define GNSS_POSER__POSITION_COVARIANCE_HPP_

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <array>
#include <cstddef>

namespace gnss_poser
{

inline std::array<double, 3> resolvePositionCovariance(
  const sensor_msgs::msg::NavSatFix & fix, const double unknown_position_covariance_m2)
{
  if (
    fix.position_covariance_type !=
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    return {
      fix.position_covariance[0], fix.position_covariance[4], fix.position_covariance[8]};
  }
  return {
    unknown_position_covariance_m2, unknown_position_covariance_m2,
    unknown_position_covariance_m2};
}

}  // namespace gnss_poser

#endif  // GNSS_POSER__POSITION_COVARIANCE_HPP_
