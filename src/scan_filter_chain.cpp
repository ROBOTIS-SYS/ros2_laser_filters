/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <vector>

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "ros2_laser_filters/filter_base.hpp"
#include "ros2_laser_filters/range_filter.hpp"
#include "ros2_laser_filters/scan_shadows_filter.hpp"
#include "ros2_laser_filters/box_filter.hpp"
#include "ros2_laser_filters/radius_search_filter.hpp"
#include "ros2_laser_filters/pose_filter.hpp"


class ScanToScanFilterChain : public rclcpp::Node
{
protected:
  // Components for tf::MessageFilter
  rclcpp::Node::SharedPtr tf_node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf_filter_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  double tf_filter_tolerance_;

  // Components for publishing
  // sensor_msgs::LaserScan msg_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  std::vector<std::shared_ptr<filters::FilterBase<sensor_msgs::msg::LaserScan>>> filters_;

  // TODO erase
  std::shared_ptr<laser_filters::PoseFilter> pointcloud_filter_;

public:
  // Constructor
  ScanToScanFilterChain()
  : Node("scan_to_scan_filter_chain"),
    tf_node_(new rclcpp::Node("scan_to_scan_filter_chain_tf")),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, tf_node_)
  {

    // declare tf target frame name parameter
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = "tf_message_filter_target_frame";
    desc.read_only = true;
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    std::string tf_target_frame = declare_parameter(desc.name, "", desc);

    if (tf_target_frame.empty()) {
      throw std::runtime_error("todo sloretz bypass tf message filter");
    }
    RCLCPP_INFO(get_logger(), "Targeting frame '%s'", tf_target_frame.c_str());

    // Setup timer creation for buffer to make waitForTransform work as used by message filters
    auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
    tf2_ros::Buffer buffer(get_clock());
    tf_buffer_.setCreateTimerInterface(create_timer_interface);

    rclcpp::SensorDataQoS sensor_qos;
    sensor_qos.keep_last(1);
    scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
      this, "scan", sensor_qos.get_rmw_qos_profile());

    tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *scan_sub_,
        tf_buffer_,
        tf_target_frame,
        50,
        get_node_logging_interface(),
        get_node_clock_interface());

    filters_.push_back(std::make_shared<laser_filters::ScanShadowsFilter>());
    filters_.back()->configure(
        "scan_chain",
        "scan_shadows_filter",
        get_node_logging_interface(),
        get_node_parameters_interface());

    filters_.push_back(std::make_shared<laser_filters::RadiusSearchFilter>());
    filters_.back()->configure(
        "scan_chain",
        "radius_search_filter",
        get_node_logging_interface(),
        get_node_parameters_interface());

    filters_.push_back(std::make_shared<laser_filters::LaserScanRangeFilter>());
    filters_.back()->configure(
        "scan_chain",
        "laser_scan_range_filter",
        get_node_logging_interface(),
        get_node_parameters_interface());

    filters_.push_back(std::make_shared<laser_filters::LaserScanBoxFilter>());
    filters_.back()->configure(
        "scan_chain",
        "laser_scan_box_filter",
        get_node_logging_interface(),
        get_node_parameters_interface());

    pointcloud_filter_ = std::make_shared<laser_filters::PoseFilter>();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", sensor_qos,
      std::bind(&ScanToScanFilterChain::odom_callback, this, std::placeholders::_1));

    // Configure filter chain
    // params
    //  tf_message_filter_target_frame
    //  tf_message_filter_tolerance
    //  private_nh_.param("", tf_filter_tolerance_, 0.03);

    //  tf_ = new tf::TransformListener();
    //  tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(scan_sub_, *tf_, "", 50);
    //  tf_filter_->setTargetFrame(tf_message_filter_target_frame);
    //  tf_filter_->setTolerance(ros::Duration(tf_filter_tolerance_));

    // Setup tf::MessageFilter generates callback
    tf_filter_->registerCallback(
        std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
    // }
    // else
    // {
    //   // Pass through if no tf_message_filter_target_frame
    //   scan_sub_.registerCallback(boost::bind(&ScanToScanFilterChain::callback, this, _1));
    // }

    // Advertise output
    // output_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_filtered", 1000);

    rclcpp::QoS qos(1000);
    output_pub_ = rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(*this, "scan_filtered", qos);
    point_cloud_pub_ = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(*this, "pc_filtered", qos);
  }

  // Destructor
  ~ScanToScanFilterChain()
  {
  }

  // Callback
  void callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg_in)
  {
    sensor_msgs::msg::LaserScan filter_input;
    sensor_msgs::msg::LaserScan filter_output(*msg_in);
    for (auto & filter : filters_) {
      filter_input = filter_output;
      filter_output = sensor_msgs::msg::LaserScan();
      if (!filter->update(filter_input, filter_output)) {
        RCLCPP_ERROR(
          get_logger(),
          "Filtering the scan from time %i.%i failed.",
          msg_in->header.stamp.sec,
          msg_in->header.stamp.nanosec);
        return;
      }
    }

    sensor_msgs::msg::PointCloud2 filtered_pc;
    pointcloud_filter_->update(filter_output, filtered_pc);

    output_pub_->publish(filter_output);
    point_cloud_pub_->publish(filtered_pc);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_in)
  {
    pointcloud_filter_->add_pose(*msg_in);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanToScanFilterChain>());
  rclcpp::shutdown();
  return 0;
}
