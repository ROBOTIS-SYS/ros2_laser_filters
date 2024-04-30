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

#include <queue>

#include <ros2_laser_filters/filter_base.hpp>

#include <laser_geometry/laser_geometry.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

  bool push_pose(const nav_msgs::msg::Odometry & odom);

private:

  std::unique_ptr<PosePredictorBase> pose_buffer_;
  laser_geometry::LaserProjection projector_;

};
class PosePredictorBase
{
public:
  PosePredictorBase();
  virtual ~PosePredictorBase();

  void predict(
    const std::vector<rclcpp::Time> & scanning_times,
    std::vector<geometry_msgs::msg::TransformStamped> & poses_out);

  inline void add_pose(geometry_msgs::msg::TransformStamped in) {pose_buff_.push(in);}

private:
  /**
   * @brief Set the pose of the last vector as reference frame.
   * Then, express every pose in the vector to a reference frame.
   *
   * @param pose_in
   * @param pose_out
   */
  void backpropagate_pose(
    const std::vector<geometry_msgs::msg::TransformStamped> & pose_in,
    std::vector<geometry_msgs::msg::TransformStamped> & pose_out);

  /**
   * @brief predict the pose on time "time_des" using the linear interpolation of odom_a and odom_b
   *  and output the predicted pose to "odom_out"
   *
   * @param time_des
   * @param odom_a
   * @param odom_b
   * @param odom_out
   */
  void interpolate_pose(
    const rclcpp::Time & time_des,
    const geometry_msgs::msg::TransformStamped & odom_a,
    const geometry_msgs::msg::TransformStamped & odom_b,
    geometry_msgs::msg::TransformStamped & odom_out);

  void interpolate_poses(
    const std::vector<rclcpp::Time> & time_des,
    const std::vector<geometry_msgs::msg::TransformStamped> & pose_in,
    std::vector<geometry_msgs::msg::TransformStamped> & pose_out);

  void pop_and_sort_buffer(rclcpp::Time latest,
    std::vector<geometry_msgs::msg::TransformStamped> & pose_out);

private:
  std::queue<geometry_msgs::msg::TransformStamped> pose_buff_;
};
}  // namespace laser_filters
#endif
