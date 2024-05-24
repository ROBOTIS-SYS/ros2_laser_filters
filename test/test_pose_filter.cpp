#include <ros2_laser_filters/pose_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

// test utils
inline bool fequal(float a, float b, float th = 1e-8) {
  return (std::fabs(a - b) < th);
}

int main(int argc, char* argv[])
{
  std::cout << "\n[INFO] ========= start test of class PoseFilter =========== \n" << std::endl;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pose_filter_test_node");
  rclcpp::Time ref_time = node->get_clock()->now();
  std::cout << "[INFO] test starting time : " << static_cast<int>(ref_time.seconds()) << std::endl;
  // create interpolation fixture
  std::vector<geometry_msgs::msg::TransformStamped> odom_set;

  /** test parameters **/
  std::string frame_id = "odom";
  std::string child_frame_id = "base_link";

  size_t test_size = 4;
  double rate_max = 1.0;
  double rate_min = 0.0;

  double due_min_s = 5.0;
  double due_max_s = 4.0;
  double x_fix, y_fix, z_fix, pt_plus;

  x_fix = 0.0;
  y_fix = 0.0;
  z_fix = 0.0;
  pt_plus = 1.3412314562346;

  const double RATE_OFFSET = (rate_max + rate_min) * 0.5;
  const double RATE_MAG = (rate_max - rate_min) * 0.5;
  const double DUE_OFFSET = (due_max_s + due_min_s) * 0.5;
  const double DUE_MAG = (due_max_s - due_min_s) * 0.5;

  // dataset generation
  geometry_msgs::msg::TransformStamped first_pose;
  first_pose.header.frame_id = frame_id;
  first_pose.child_frame_id = child_frame_id;
  first_pose.header.stamp = ref_time;
  first_pose.transform.translation.x = x_fix;
  first_pose.transform.translation.y = y_fix;
  first_pose.transform.translation.z = z_fix;

  odom_set.push_back(first_pose);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  for (size_t test_no = 0; test_no < test_size - 1; test_no++) {
    Eigen::Isometry3d pose_increment = Eigen::Isometry3d::Identity();
    pose_increment.translate(Eigen::Vector3d(2 * pt_plus, - pt_plus, 0));
    auto axis = Eigen::Vector3d(0, 0, 1);
    axis.normalize();
    pose_increment.rotate(Eigen::AngleAxisd(0.128, axis));

    pose = pose * pose_increment;

    double duration_sec = DUE_OFFSET + DUE_MAG * std::sin(M_2_PI * test_no / 50);

    geometry_msgs::msg::TransformStamped odom = odom_set.back();
    odom.header.stamp = rclcpp::Time(odom.header.stamp) +
      rclcpp::Duration::from_seconds(duration_sec);

    geometry_msgs::msg::Pose pose_msg = tf2::toMsg(pose);
    odom.transform.rotation = pose_msg.orientation;
    odom.transform.translation.x = pose_msg.position.x;
    odom.transform.translation.y = pose_msg.position.y;
    odom.transform.translation.z = pose_msg.position.z;

    odom_set.push_back(odom);
  }
  // create pose_filter object
  std::shared_ptr<laser_filters::PoseFilter> test_candidate_filter;
  test_candidate_filter = std::make_shared<laser_filters::PoseFilter>();

  // register pose to filter
  for (auto it = odom_set.begin(); it != odom_set.end(); it++) {
    test_candidate_filter->add_pose(*it);
  }

  // create arbitrary pointcloud
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  // set scan parameter
  size_t pt_size = 100;
  const double FIXED_RANGE = 1000.0;

  // create arbitrary pointcloud
  double angle_increment = M_PI * 2 / pt_size;
  for (size_t i = 0; i < pt_size; i++) {
    double angle = angle_increment * i;
    pcl::PointXYZ pt;
    pt.x = FIXED_RANGE * std::cos(angle);
    pt.y = FIXED_RANGE * std::sin(angle);
    pt.z = 0.0;
    point_cloud.push_back(pt);
  }

  // create interplated motion
  rclcpp::Time time_begin = rclcpp::Time(odom_set.begin()->header.stamp) -
    rclcpp::Duration::from_seconds(2.0);

  rclcpp::Time time_end = rclcpp::Time(odom_set.end()->header.stamp) +
    rclcpp::Duration::from_seconds(2.0);

  rclcpp::Duration total_duration = time_end - time_begin;

  std::vector<rclcpp::Time> scan_dev_stamps;
  for (size_t i = 0; i < pt_size; i++) {
    scan_dev_stamps.push_back(time_begin +
      (total_duration * (1 / static_cast<double>(pt_size - 1))) * static_cast<double>(i));
  }

  std::vector<geometry_msgs::msg::TransformStamped> pose_intpl;
  test_candidate_filter->pose_buffer_->interpolate_poses(scan_dev_stamps, pose_intpl);

  // create disorted scan
  sensor_msgs::msg::LaserScan laser_msgs;
  laser_msgs.angle_increment = 2 * M_PI / (pt_size);
  laser_msgs.angle_min = 0.0;
  laser_msgs.angle_max = 2 * M_PI * (1 - 1 / (pt_size));
  laser_msgs.time_increment = total_duration.seconds() * (1 / static_cast<double>(pt_size - 1));

  laser_msgs.header.frame_id = "lidar_link";
  laser_msgs.header.stamp = time_end;

  // fill in the laser_scan container
  laser_msgs.ranges.reserve(pt_size);
  laser_msgs.intensities.reserve(pt_size);



  return 0;
}

