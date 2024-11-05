/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  box_filter.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include "ros2_laser_filters/box_filter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/create_timer_ros.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

laser_filters::LaserScanBoxFilter::LaserScanBoxFilter()
: tf_node_(new rclcpp::Node("laser_scan_box_filter_tf")),
  tf_buffer_(tf_node_->get_clock()),
  tf_listener_(tf_buffer_, tf_node_)
{
  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    tf_node_->get_node_base_interface(),
    tf_node_->get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(create_timer_interface);
}

bool laser_filters::LaserScanBoxFilter::configure()
{
  up_and_running_ = true;
  double min_x = 0, min_y = 0, min_z = 0, max_x = 0, max_y = 0, max_z = 0;
  bool box_frame_set = getParam("box_frame", box_frame_);
  bool x_max_set = getParam("max_x", max_x);
  bool y_max_set = getParam("max_y", max_y);
  bool z_max_set = getParam("max_z", max_z);
  bool x_min_set = getParam("min_x", min_x);
  bool y_min_set = getParam("min_y", min_y);
  bool z_min_set = getParam("min_z", min_z);
  bool invert_set = getParam("invert", invert_filter);

  auto logger = logging_interface_->get_logger().get_child(filter_name_);
  RCLCPP_INFO(logger, "BOX filter started");

  max_.setX(max_x);
  max_.setY(max_y);
  max_.setZ(max_z);
  min_.setX(min_x);
  min_.setY(min_y);
  min_.setZ(min_z);

  if (!box_frame_set) {
    RCLCPP_ERROR(logger, "box_frame is not set!");
  }
  if (!x_max_set) {
    RCLCPP_ERROR(logger, "max_x is not set!");
  }
  if (!y_max_set) {
    RCLCPP_ERROR(logger, "max_y is not set!");
  }
  if (!z_max_set) {
    RCLCPP_ERROR(logger, "max_z is not set!");
  }
  if (!x_min_set) {
    RCLCPP_ERROR(logger, "min_x is not set!");
  }
  if (!y_min_set) {
    RCLCPP_ERROR(logger, "min_y is not set!");
  }
  if (!z_min_set) {
    RCLCPP_ERROR(logger, "min_z is not set!");
  }
  if (!invert_set) {
    RCLCPP_INFO(logger, "invert filter not set, assuming false");
    invert_filter = false;
  }


  return box_frame_set && x_max_set && y_max_set && z_max_set &&
         x_min_set && y_min_set && z_min_set;

}

bool laser_filters::LaserScanBoxFilter::update(
  const sensor_msgs::msg::LaserScan & input_scan,
  sensor_msgs::msg::LaserScan & output_scan)
{
  auto logger = logging_interface_->get_logger().get_child(filter_name_);
  output_scan = input_scan;
  sensor_msgs::msg::PointCloud2 laser_cloud;


  rclcpp::Duration additional_time(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<float>(input_scan.ranges.size() * input_scan.time_increment)));

  rclcpp::Duration timeout = rclcpp::Duration(1.0);

  auto future = tf_buffer_.waitForTransform(
    box_frame_,
    input_scan.header.frame_id,
    rclcpp::Time(input_scan.header.stamp) + additional_time,
    timeout,
    [](tf2_ros::TransformStampedFuture) { /* do nothing */});

  future.wait();


  // if(!success){
  //   RCLCPP_WARN(logger, "Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
  //   return false;
  // }

  // TODO(sloretz) needs clock, give filters clock interface

  try {
    projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, tf_buffer_);
  } catch (const tf2::TransformException & ex) {
    if (up_and_running_) {
      // TODO(sloretz) uncomment when https://github.com/ros2/rclcpp/pull/981 is released
      // RCLCPP_WARN_THROTTLE(logger, clock, 1, "Dropping Scan: Tansform unavailable %s", ex.what());
      return true;
    } else {
      // TODO(sloretz) uncomment when https://github.com/ros2/rclcpp/pull/981 is released
      // RCLCPP_INFO_THROTTLE(logger, clock, .3, "Ignoring Scan: Waiting for TF");
    }
    return false;
  }
  const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
  const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
  const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
  const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

  if (i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1) {
    // TODO(sloretz) uncomment when https://github.com/ros2/rclcpp/pull/981 is released
    // RCLCPP_INFO_THROTTLE(logger, clock, .3, "x, y, z and index fields are required, skipping scan");
  }


  const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
  const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
  const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
  const int z_idx_offset = laser_cloud.fields[z_idx_c].offset;

  const int pstep = laser_cloud.point_step;
  const long int pcount = laser_cloud.width * laser_cloud.height;
  const long int limit = pstep * pcount;

  int i_idx, x_idx, y_idx, z_idx;
  for (
    i_idx = i_idx_offset,
    x_idx = x_idx_offset,
    y_idx = y_idx_offset,
    z_idx = z_idx_offset;

    x_idx < limit;

    i_idx += pstep,
    x_idx += pstep,
    y_idx += pstep,
    z_idx += pstep)
  {

    // TODO works only for float data types and with an index field
    // I'm working on it, see https://github.com/ros/common_msgs/pull/78
    float x = *((float *)(&laser_cloud.data[x_idx]));
    float y = *((float *)(&laser_cloud.data[y_idx]));
    float z = *((float *)(&laser_cloud.data[z_idx]));
    int index = *((int *)(&laser_cloud.data[i_idx]));

    tf2::Vector3 point(x, y, z);

    if (!invert_filter) {
      if (inBox(point)) {
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    } else {
      if (!inBox(point)) {
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }

  }
  up_and_running_ = true;
  return true;
}

bool laser_filters::LaserScanBoxFilter::inBox(tf2::Vector3 & point)
{
  return
    point.x() < max_.x() && point.x() > min_.x() &&
    point.y() < max_.y() && point.y() > min_.y() &&
    point.z() < max_.z() && point.z() > min_.z();
}
