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
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>

laser_filters::PoseFilter::PoseFilter(PoseFilterParams params)
{
  params_ = params;
  pose_buffer_ = std::make_unique<PosePredictorBase>();
}

laser_filters::PoseFilter::~PoseFilter() {}


// bool laser_filters::PoseFilter::update(
//   const sensor_msgs::msg::LaserScan& input_scan,
//   sensor_msgs::msg::LaserScan &output_scan)
// {
//   output_scan = input_scan;
// }

bool laser_filters::PoseFilter::update(
  const sensor_msgs::msg::LaserScan& input_scan,
  sensor_msgs::msg::PointCloud2 &output_scan)
{
  std::vector<rclcpp::Time> stamps;

  create_pt_wise_stamp(input_scan, stamps);
  input_scan.time_increment;

  std::vector<geometry_msgs::msg::TransformStamped> pose_predict;

  pose_buffer_->predict(stamps, pose_predict);

  if (params_.publish_pose_history_) {
    geometry_msgs::msg::PoseArray pose_history;
    pose_history.header.stamp = pose_predict.back().header.stamp;
    pose_history.header.frame_id = "lidar_link";
    for (auto it = pose_predict.begin(); it != pose_predict.end(); it++) {
      geometry_msgs::msg::Pose pose_cvt;
      pose_cvt.position.x = it->transform.translation.x;
      pose_cvt.position.y = it->transform.translation.y;
      pose_cvt.position.z = it->transform.translation.z;
      pose_cvt.orientation = it->transform.rotation;
      pose_history.poses.push_back(pose_cvt);
    }
  }

  if (pose_predict.empty()) return false;

  output_scan = pointwize_alignment(input_scan, pose_predict);

  return true;
}

void laser_filters::PoseFilter::add_pose(const nav_msgs::msg::Odometry & msg)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header = msg.header;
  transform_msg.child_frame_id = msg.child_frame_id;
  transform_msg.transform.rotation = msg.pose.pose.orientation;

  // position to translation
  {
    transform_msg.transform.translation.x = msg.pose.pose.position.x;
    transform_msg.transform.translation.y = msg.pose.pose.position.y;
    transform_msg.transform.translation.z = msg.pose.pose.position.z;
  }
  add_pose(transform_msg);
}

bool laser_filters::PoseFilter::create_pt_wise_stamp(const sensor_msgs::msg::LaserScan& input_scan,
  std::vector<rclcpp::Time> & stamp_out)
{
  std::vector<float> scan_container = input_scan.ranges;
  auto point_due = rclcpp::Duration::from_seconds(input_scan.time_increment);
  rclcpp::Time end_point_stamp = input_scan.header.stamp;

  std::vector<rclcpp::Time> stamp_container;

  for (int ridx = static_cast<int>(scan_container.size()) - 1; ridx >= 0; ridx--) {
    stamp_container.push_back(end_point_stamp - point_due * ridx);
  }

  stamp_out = stamp_container;
}

sensor_msgs::msg::PointCloud2 laser_filters::PoseFilter::pointwize_alignment(
  const sensor_msgs::msg::LaserScan& input_scan,
  const std::vector<geometry_msgs::msg::TransformStamped>& pointwise_pose)
{
  std::vector<float> scan_container = input_scan.ranges;
  float angle_min = input_scan.angle_min;
  float angle_increment = input_scan.angle_increment;
  size_t scan_container_size = scan_container.size();

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

  sensor_msgs::msg::PointCloud2 cloud_out;

  unsigned int count = 0;
  for (size_t idx = 0; idx < scan_container_size; idx++) {
    if (std::isnan(scan_container[idx])) continue;
    Eigen::Vector3d point(
      std::cos(angle_min + static_cast<float>(idx) * angle_increment),
      std::sin(angle_min + static_cast<float>(idx) * angle_increment),
      0.0);

    point *= scan_container[idx];

    auto pose = pointwise_pose[idx];
    Eigen::Vector3d point_out;
    tf2::doTransform(point, point_out, pose);

    pcl::PointXYZ pcl_pt;
    pcl_pt.x = point_out(0); pcl_pt.y = point_out(1); pcl_pt.z = point_out(2);
    pcl_cloud.push_back(pcl_pt);
  }

  // resize if necessary
  pcl::toROSMsg(pcl_cloud, cloud_out);
  cloud_out.header = input_scan.header;

  return cloud_out;
}


laser_filters::PosePredictorBase::PosePredictorBase() {}
laser_filters::PosePredictorBase::~PosePredictorBase() {}

void laser_filters::PosePredictorBase::predict(
  const std::vector<rclcpp::Time> & scanning_times,
  std::vector<geometry_msgs::msg::TransformStamped> & poses_out)
{
  std::vector<geometry_msgs::msg::TransformStamped> poses_interpol;

  interpolate_poses(scanning_times, poses_interpol);

  std::vector<geometry_msgs::msg::TransformStamped> pose_backprop;
  backpropagate_pose(poses_interpol, pose_backprop);

  poses_out = pose_backprop;
  pop_buffer(scanning_times.back());
}

void laser_filters::PosePredictorBase::add_pose(geometry_msgs::msg::TransformStamped in)
{
  if (pose_buff_.empty()) {
    pose_buff_.push_back(in);
  } else if (rclcpp::Time(in.header.stamp) > pose_buff_.back().header.stamp)  {
    pose_buff_.push_back(in);
  } else {
    auto it = pose_buff_.rbegin();
    while (it != pose_buff_.rend() && rclcpp::Time(it->header.stamp) > in.header.stamp) {
      ++it;
    }
    pose_buff_.insert(it.base(), in);
  }

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
  std::vector<geometry_msgs::msg::TransformStamped> pose_temp;

  // Set the last pose as the reference frame
  const auto& reference_pose = pose_in.back();

  // Initialize the output vector with the same size as pose_in
  pose_temp.reserve(pose_in.size());

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
    pose_temp.push_back(relative_transform);
  }
  // return value
  pose_out = pose_temp;
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

  // You can add your implementation here.
}

bool laser_filters::PosePredictorBase::interpolate_poses(
  const std::vector<rclcpp::Time> & time_des,
  std::vector<geometry_msgs::msg::TransformStamped> & pose_out)
{
  // pose_in의 길이가 2 이하일 경우 예외처리
  if (time_des.empty() || pose_buff_.empty()) { return false; }

  std::vector<geometry_msgs::msg::TransformStamped> pose_vec; // output container
  pose_vec.reserve(time_des.size());

  if (pose_buff_.size() < 2) {  // cannot interpolate, need at least two poses
    for (auto tit = time_des.begin(); tit != time_des.end(); tit++) {
      geometry_msgs::msg::TransformStamped t_in = pose_buff_.front();
      t_in.header.stamp = *tit;
      pose_vec.push_back(t_in);
    }
    pose_out = pose_vec;
    return true;
  }

  for (auto tit = time_des.begin(); tit != time_des.end(); tit++) {
    auto pit = ++(pose_buff_.begin());
    for(; pit != pose_buff_.end(); pit++) {
      if (*tit < pit->header.stamp) { break; }
    }
    if (pit == pose_buff_.end()) { pit--; }
    geometry_msgs::msg::TransformStamped pose_interpl;
    interpolate_pose(*tit, *(pit - 1), *pit, pose_interpl);
    pose_vec.push_back(pose_interpl);
  }
  pose_out = pose_vec;

  return true;
}

void laser_filters::PosePredictorBase::pop_buffer(rclcpp::Time latest)
{
  // clone
  if (!empty()) {
    size_t idx_holder = 0;
    for (; idx_holder < pose_buff_.size() - 2; idx_holder++) {
      if (latest < pose_buff_[idx_holder].header.stamp) {
        break;
      }
    }

    for (size_t idx = 0; idx < idx_holder; idx++) {
      pose_buff_.pop_front();
    }
  }
}

