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

#include <algorithm>

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

void laser_filters::PosePredictorBase::predict(
  const std::vector<rclcpp::Time> & scanning_times,
  std::vector<geometry_msgs::msg::TransformStamped> & poses_out)
{
  std::vector<geometry_msgs::msg::TransformStamped> poses_sort;

  pop_and_sort_buffer(scanning_times.back(), poses_sort);

  std::vector<geometry_msgs::msg::TransformStamped> poses_interpol;

  interpolate_poses(scanning_times, poses_sort, poses_interpol);
  backpropagate_pose(poses_interpol, poses_out);
}

void laser_filters::PosePredictorBase::backpropagate_pose(
  const std::vector<geometry_msgs::msg::TransformStamped> & pose_in,
  std::vector<geometry_msgs::msg::TransformStamped> & pose_out)
{
// Ensure that there are at least two poses in the input vector
  if (pose_in.size() < 2) {
    // Handle the case where there are insufficient poses
    // (e.g., throw an exception or return an error code)
    return;
  }

  // Set the last pose as the reference frame
  const auto& reference_pose = pose_in.back();

  // Initialize the output vector with the same size as pose_in
  pose_out.resize(pose_in.size());

  // Iterate through each pose in pose_in
  for (size_t i = 0; i < pose_in.size(); ++i) {
    // Calculate the relative transform from the reference pose
    const auto& current_pose = pose_in[i];
    geometry_msgs::msg::TransformStamped relative_transform;

    // Calculate relative translation
    relative_transform.transform.translation.x =
        current_pose.transform.translation.x - reference_pose.transform.translation.x;
    relative_transform.transform.translation.y =
        current_pose.transform.translation.y - reference_pose.transform.translation.y;
    relative_transform.transform.translation.z =
        current_pose.transform.translation.z - reference_pose.transform.translation.z;

    // Calculate relative rotation (quaternion difference)
    tf2::Quaternion q_a, q_b, q_relative;
    tf2::fromMsg(current_pose.transform.rotation, q_a);
    tf2::fromMsg(reference_pose.transform.rotation, q_b);

    // Compute the relative rotation quaternion
    q_relative = q_b.inverse() * q_a;

    // Rotate the relative translation using the relative rotation
    tf2::Matrix3x3 rotation_matrix(q_b.inverse());
    tf2::Vector3 rotated_translation = rotation_matrix * tf2::Vector3(
        relative_transform.transform.translation.x,
        relative_transform.transform.translation.y,
        relative_transform.transform.translation.z);

    // Set the rotated translation in relative_transform
    relative_transform.transform.translation.x = rotated_translation.x();
    relative_transform.transform.translation.y = rotated_translation.y();
    relative_transform.transform.translation.z = rotated_translation.z();

    // Convert the relative rotation quaternion back to geometry_msgs format
    relative_transform.transform.rotation = tf2::toMsg(q_relative);

    // Set other fields in relative_transform (e.g., child_frame_id)
    relative_transform.header.frame_id = pose_in.back().child_frame_id; // Set your desired frame ID

    // Store the relative transform in pose_out
    pose_out[i] = relative_transform;
  }
}

void laser_filters::PosePredictorBase::interpolate_pose(
  const rclcpp::Time & time_des,
  const geometry_msgs::msg::TransformStamped & odom_a,
  const geometry_msgs::msg::TransformStamped & odom_b,
  geometry_msgs::msg::TransformStamped & odom_out)
{
   // Extract relevant data from odom_a and odom_b
  const auto& position_a = odom_a.transform.translation;
  tf2::Quaternion orientation_a(
    odom_a.transform.rotation.x,
    odom_a.transform.rotation.y,
    odom_a.transform.rotation.z,
    odom_a.transform.rotation.w);

  rclcpp::Time time_a = odom_a.header.stamp;

  const auto& position_b = odom_b.transform.translation;
  tf2::Quaternion orientation_b(
    odom_b.transform.rotation.x,
    odom_b.transform.rotation.z,
    odom_b.transform.rotation.y,
    odom_b.transform.rotation.w);

  rclcpp::Time time_b = odom_b.header.stamp;

  // Calculate interpolation factor (0 to 1) based on time_des
  double t_factor = (time_des - time_a).seconds() / (time_b - time_a).seconds();

  // Linearly interpolate position
  odom_out.transform.translation.x = position_a.x + t_factor * (position_b.x - position_a.x);
  odom_out.transform.translation.y = position_a.y + t_factor * (position_b.y - position_a.y);
  odom_out.transform.translation.z = position_a.z + t_factor * (position_b.z - position_a.z);

  // Slerp interpolate orientation (use quaternion slerp)
  tf2::Quaternion orientation_out = slerp(orientation_a, orientation_b, t_factor);
  odom_out.transform.rotation.x = orientation_out.getX();
  odom_out.transform.rotation.y = orientation_out.getY();
  odom_out.transform.rotation.z = orientation_out.getZ();
  odom_out.transform.rotation.w = orientation_out.getW();

  // Set other fields in odom_out (e.g., child_frame_id, header.stamp)
  odom_out.header.frame_id = odom_b.header.frame_id;
  odom_out.header.stamp = time_des;

  // You can add your implementation here
}

void laser_filters::PosePredictorBase::interpolate_poses(
  const std::vector<rclcpp::Time> & time_des,
  const std::vector<geometry_msgs::msg::TransformStamped> & pose_in,
  std::vector<geometry_msgs::msg::TransformStamped> & pose_out)
{

  std::vector<geometry_msgs::msg::TransformStamped> pose_vec;
  pose_vec.reserve(time_des.size());

  // TODO: pose_in의 길이가 1이하일 경우 예외처리 필수
  auto pit = ++(pose_in.begin());
  for (auto it = time_des.begin(); it != time_des.end(); it++) {
    for(; pit != pose_in.end(); pit++) {
      if (*it < pit->header.stamp) { break; }
    }
    geometry_msgs::msg::TransformStamped pose_interpl;
    interpolate_pose(*it, *(--pit), *pit, pose_interpl);
    pose_vec.push_back(pose_interpl);
  }
  pose_out = pose_vec;
}

void laser_filters::PosePredictorBase::pop_and_sort_buffer(rclcpp::Time latest,
  std::vector<geometry_msgs::msg::TransformStamped> & pose_out)
{
  std::vector<geometry_msgs::msg::TransformStamped> pose_vec;

  // pop
  while(!pose_buff_.empty()) {
    if (latest > pose_buff_.front().header.stamp) {
      pose_vec.push_back(pose_buff_.front());
      pose_buff_.pop();
    } else {
      break;
    }
  }

  // sort
  auto time_comp = [](
    const geometry_msgs::msg::TransformStamped& a,
    const geometry_msgs::msg::TransformStamped& b)
    {
      return rclcpp::Time(a.header.stamp) < rclcpp::Time(b.header.stamp);
    };

  std::sort(pose_vec.begin(), pose_vec.end(), time_comp);

  if( pose_buff_.empty() && !pose_vec.empty()) {
    pose_buff_.push(pose_vec.back()); // save the latest
  }

  pose_out = pose_vec; // output
}

