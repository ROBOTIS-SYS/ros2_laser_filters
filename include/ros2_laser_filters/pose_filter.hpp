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

#ifndef POSE_FILTER_HPP_
#define POSE_FILTER_HPP_
#include <ros2_laser_filters/filter_base.hpp>

#include <laser_geometry/laser_geometry.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

namespace laser_filters {

class PoseFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:

  PoseFilter();

  bool configure();

  bool update(
    const sensor_msgs::msg::LaserScan& input_scan,
    sensor_msgs::msg::LaserScan& filtered_scan);

  bool push_to_imu_buffer(const sensor_msgs::msg::Imu & imu_buff);
  bool push_to_odom_buffer(const nav_msgs::msg::Odometry::SharedPtr & odom_buff);


private:
  std::unique_ptr<PosePredictorBase> pose_buffer_;

};

class PosePredictorBase
{
public:
  PosePredictorBase();
  ~PosePredictorBase() = delete;

  void update_buffer_with_interpolation(std::vector<geometry_msgs::msg::TransformStamped> & pose_out);

  inline void add_pose(geometry_msgs::msg::TransformStamped in) {pose_buff_.push_back(in);}

private:
  void free_unused_buffer(); // used in update_buffer_with interpolation

  std::vector<geometry_msgs::msg::TransformStamped> pose_buff_;
}

}  // namespace laser_filters
#endif
