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

bool laser_filters::PoseFilter::update(
  const sensor_msgs::msg::LaserScan& input_scan,
  sensor_msgs::msg::PointCloud2 &output_scan)
{
  std::vector<rclcpp::Time> stamps;

  create_pt_wise_stamp(input_scan, stamps);
  input_scan.time_increment;

  std::vector<geometry_msgs::msg::TransformStamped> pose_predict;

  pose_buffer_->predict(stamps, pose_predict);

  if (params_.publish_pose_history_ && !pose_predict.empty()) {
    geometry_msgs::msg::PoseArray pose_history;

    // TODO(SangBeom Woo) : 취약점 해결 필요
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
  const std::vector<geometry_msgs::msg::TransformStamped>& pointwise_transform)
{
  // a. get laser scan header parameters;
  float angle_min = input_scan.angle_min;
  float angle_increment = input_scan.angle_increment;
  size_t scan_container_size = input_scan.ranges.size();

  // b. output conversion interface
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud; // output pointcloud
  sensor_msgs::msg::PointCloud2 cloud_out; // output pointcloud message

  // c. convert from LaserScan to motion-distortion-free PointCloud (process 1 to 6)
  for (size_t idx = 0; idx < scan_container_size; idx++) {
    // c - 1. check number value and drop if nan
    if (std::isnan(input_scan.ranges[idx])) continue;
    // c - 2. create ray-of-light unit vector from azimuth
    Eigen::Vector3d point_in(
      std::cos(angle_min + static_cast<float>(idx) * angle_increment),
      std::sin(angle_min + static_cast<float>(idx) * angle_increment), 0.0);
    // c - 3. multiply scan-range so that the unit vector become point in pointwize-frame.
    point_in *= input_scan.ranges[idx];
    // c - 4. get reference-frame to pointwize-frame.
    geometry_msgs::msg::TransformStamped transform = pointwise_transform[idx];
    Eigen::Vector3d point_out;
    // c - 5. transform the point from pointwize-frame to reference-frame.
    tf2::doTransform(point_in, point_out, transform);
    // c - 6. convert eigen to pcl::PointXYZI.
    pcl::PointXYZI pcl_pt;
    pcl_pt.x = point_out(0); pcl_pt.y = point_out(1); pcl_pt.z = point_out(2); // point conversion
    pcl_pt.intensity = input_scan.intensities[idx]; // intensity conversion
    pcl_cloud.push_back(pcl_pt);
  }

  // d. convert to ROSMsg
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
  Eigen::Isometry3d reference_pose_eigen;
  reference_pose_eigen = tf2::transformToEigen(reference_pose);

  // Initialize the output vector with the same size as pose_in
  pose_temp.reserve(pose_in.size());

  // Iterate through each pose in pose_in
  for (size_t i = 0; i < pose_in.size(); ++i) {
    // Calculate the relative transform from the reference pose
    Eigen::Isometry3d current_pose_eigen;
    current_pose_eigen = tf2::transformToEigen(pose_in[i]);

    Eigen::Isometry3d relative_transform_eigen =
      reference_pose_eigen.inverse() * current_pose_eigen;

    geometry_msgs::msg::TransformStamped relative_transform =
      tf2::eigenToTransform(relative_transform_eigen);

    // Set other fields in relative_transform (e.g., child_frame_id)
    relative_transform.header.frame_id = pose_in.back().child_frame_id;
    // Store the relative transform in pose_out
    pose_temp.push_back(relative_transform);
  }

  // return value
  pose_out = pose_temp;
}

 geometry_msgs::msg::TransformStamped laser_filters::PosePredictorBase::interpolate_pose(
  const rclcpp::Time & time_des,
  const geometry_msgs::msg::TransformStamped & odom_a,
  const geometry_msgs::msg::TransformStamped & odom_b)
{
  // a - 1. Extract relevant data from odom_a
  const auto& position_a = odom_a.transform.translation;
  tf2::Quaternion orientation_a;
  tf2::fromMsg(odom_a.transform.rotation, orientation_a); // to use function slerp
  rclcpp::Time time_a = odom_a.header.stamp;

  // a - 2. Extract relevant data from odom_b
  const auto& position_b = odom_b.transform.translation;
  tf2::Quaternion orientation_b;
  tf2::fromMsg(odom_b.transform.rotation, orientation_b); // to use function slerp
  rclcpp::Time time_b = odom_b.header.stamp;

  // b. Calculate interpolation factor (0 to 1) based on time_des
  double ratio = (time_des - time_a).seconds() / (time_b - time_a).seconds();

  // c. - 1. Linearly interpolate position
  geometry_msgs::msg::TransformStamped odom_out;
  geometry_msgs::msg::Vector3 translation;

  translation.x = position_a.x + ratio * (position_b.x - position_a.x);
  translation.y = position_a.y + ratio * (position_b.y - position_a.y);
  translation.z = position_a.z + ratio * (position_b.z - position_a.z);

  odom_out.transform.translation = translation;

  // c. - 2.  Slerp interpolate orientation (use quaternion slerp)
  tf2::Quaternion orientation_out = slerp(orientation_a, orientation_b, ratio);
  odom_out.transform.rotation = tf2::toMsg(orientation_out);

  // d. Set other fields in odom_out (e.g., child_frame_id, header.stamp)
  odom_out.header.frame_id = odom_b.header.frame_id;
  odom_out.header.stamp = time_des;
  return odom_out;
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
    pose_vec.push_back(interpolate_pose(*tit, *(pit - 1), *pit));
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

