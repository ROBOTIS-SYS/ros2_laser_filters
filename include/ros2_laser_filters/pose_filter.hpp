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

#include <deque>

#include <ros2_laser_filters/filter_base.hpp>

#include <laser_geometry/laser_geometry.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>


namespace laser_filters {

class PosePredictorBase
{
public:
  PosePredictorBase();
  ~PosePredictorBase();

  void predict(
    const std::vector<rclcpp::Time> & scanning_times,
    std::vector<geometry_msgs::msg::TransformStamped> & poses_out);

  void add_pose(geometry_msgs::msg::TransformStamped in);

  inline bool can_predict() {return (pose_buff_.size() > 1);}
  inline bool empty() {return pose_buff_.empty();}



public:
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
  geometry_msgs::msg::TransformStamped interpolate_pose(
    const rclcpp::Time & time_des,
    const geometry_msgs::msg::TransformStamped & odom_a,
    const geometry_msgs::msg::TransformStamped & odom_b);

  bool interpolate_poses(
    const std::vector<rclcpp::Time> & time_des,
    std::vector<geometry_msgs::msg::TransformStamped> & pose_out);

  void pop_buffer(rclcpp::Time latest);

  inline std::deque<geometry_msgs::msg::TransformStamped> get_pose_buffer() { return pose_buff_;}
private:
  std::deque<geometry_msgs::msg::TransformStamped> pose_buff_;
};

struct PoseFilterParams
{
  bool publish_pose_history_ = false;
};

class PoseFilter
{
public:

  PoseFilter(PoseFilterParams params = PoseFilterParams());
  ~PoseFilter();

  bool update(
    const sensor_msgs::msg::LaserScan& input_scan,
    sensor_msgs::msg::PointCloud2& filtered_scan);

  // bool update(
  //   const sensor_msgs::msg::LaserScan& input_scan,
  //   sensor_msgs::msg::LaserScan& filtered_scan);

  inline void add_pose(const geometry_msgs::msg::TransformStamped & msg)
  { pose_buffer_->add_pose(msg); }

  inline geometry_msgs::msg::PoseArray get_pose_history() {return pose_history_;}

  void add_pose(const nav_msgs::msg::Odometry & msg);
  inline const PoseFilterParams get_params() {return params_;}

  std::unique_ptr<PosePredictorBase> pose_buffer_;

private:

  bool create_pt_wise_stamp(const sensor_msgs::msg::LaserScan& input_scan,
    std::vector<rclcpp::Time> & stamp_out);

  /**
   * @brief undistort the LaserScan motion-distortion to pointcloud message, given that
   * pointwise lidar base poses are present.
   *
   * @param input_scan LaserScan suffering from motion-distortion.
   * @param pointwise_pose vector size must be equal to scan size of input_scan.
   * @return sensor_msgs::msg::PointCloud2
   */
  sensor_msgs::msg::PointCloud2 pointwize_alignment(
    const sensor_msgs::msg::LaserScan& input_scan,
    const std::vector<geometry_msgs::msg::TransformStamped>& pointwise_pose);

  geometry_msgs::msg::PoseArray pose_history_;
  PoseFilterParams params_;
};
}  // namespace laser_filters
#endif
