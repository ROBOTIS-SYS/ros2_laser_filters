// Copyright 2024 ROBOTIS CO., LTD.
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
//
// Author: SangBeom Woo

#include "ros2_laser_filters/pose_filter.hpp"

laser_filters::PoseFilter::PoseFilter() {}

laser_filters::PoseFilter::~PoseFilter() {}

bool laser_filters::PoseFilter::configure()
{
  // Setup default values
  // param_ = 1;

  // launch values from parameter server
  // getParam("param", param_);

  return true;
}

bool laser_filters::PoseFilter::update(
  const sensor_msgs::msg::LaserScan& input_scan,
  sensor_msgs::msg::LaserScan &output_scan)
{
  return true;
}

laser_filters::PosePredictorBase::PosePredictorBase() {}

void laser_filters::PosePredictorBase::update_buffer_with_interpolation(
  const sensor_msgs::msg::LaserScan& input_scan,
  std::vector<geometry_msgs::msg::TransformStamped> & pose_out)
{
  std::vector<geometry_msgs::msg::TransformStamped> pose_queue;
  // do interpolation with input_scan and pose buffer and output queue
  // 1. pop and copy the queue
  // 2. interpolate
  // 3. return
  pose_out = pose_queue;

}
