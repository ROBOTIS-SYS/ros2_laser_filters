#include <ros2_laser_filters/pose_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <builtin_interfaces/msg/time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

// test utils
inline bool fequal(double a, double b, double th = 1e-5) {
  return (std::fabs(a - b) < th);
}

void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {

    int N              = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int         i = 0; i < N; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

int main(int argc, char* argv[])
{
  std::cout << "\n[INFO] ========= start test of class PoseFilter =========== \n" << std::endl;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pose_filter_test_node");
  builtin_interfaces::msg::Time ref_time = node->get_clock()->now();
  std::cout << "[INFO] test starting time : " << ref_time.nanosec << std::endl;
  // create interpolation fixture
  std::vector<geometry_msgs::msg::TransformStamped> odom_set;

  /** test parameters **/
  std::string frame_id = "odom";
  std::string child_frame_id = "base_link";

  size_t test_size = 15;
  double rate_max = 1.0;
  double rate_min = 0.0;

  double due_min_s = 5.0;
  double due_max_s = 4.0;
  double x_fix, y_fix, z_fix, pt_plus;

  x_fix = 0.0;
  y_fix = -0.6;
  z_fix = 0.0;
  pt_plus = 0.06;

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

  Eigen::Isometry3d pose = tf2::transformToEigen(first_pose);

  for (size_t test_no = 0; test_no < test_size - 1; test_no++) {
    double a = 0.33;
    double b = 1;
    if (test_no > test_size/2) b = -1;
    a *= b;
    Eigen::Isometry3d pose_increment = Eigen::Isometry3d::Identity();
    pose_increment.translate(Eigen::Vector3d(pt_plus, 0.5 * pt_plus * b, 0.0));
    auto axis = Eigen::Vector3d(0, 0, 1);
    axis.normalize();

    pose_increment.rotate(Eigen::AngleAxisd(a, axis));

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

  // set scan parameter
  size_t pt_size = 100;
  const double FIXED_RANGE = 1.0;

  // create interplated motion
  builtin_interfaces::msg::Time time_begin = rclcpp::Time(odom_set.front().header.stamp) -
    rclcpp::Duration::from_seconds(2.0);

  builtin_interfaces::msg::Time time_end = rclcpp::Time(odom_set.back().header.stamp) +
    rclcpp::Duration::from_seconds(2.0);

  rclcpp::Duration total_duration = rclcpp::Time(time_end) - time_begin;

  std::vector<builtin_interfaces::msg::Time> scan_dev_stamps;
  for (size_t i = 0; i < pt_size; i++) {
    scan_dev_stamps.push_back(rclcpp::Time(time_begin) +
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

  for (size_t idx = 0; idx < pt_size; idx++) {
    double angle = laser_msgs.angle_min + static_cast<double>(idx) * laser_msgs.angle_increment;
    tf2::Quaternion q;
    tf2::fromMsg(pose_intpl[idx].transform.rotation, q);
    double x = pose_intpl[idx].transform.translation.x;
    double y = pose_intpl[idx].transform.translation.y;

    q.normalize();
    double global_angle = angle + q.getAxis().getZ() * q.getAngle();

    if (global_angle > 2 * M_PI) {
      global_angle -= 2 * M_PI;
    } else if (global_angle < 0.0) {
      global_angle += 2 * M_PI;
    }
    // form a quadratic equation as^2 + bs + c = 0, solve s
    // problem definition
    double a = 1.0;
    double b = 2 * (y * std::sin(global_angle) + x * std::cos(global_angle));
    double c = std::pow(x, 2) + std::pow(y, 2) - std::pow(FIXED_RANGE, 2);

    // solver
    double s = (- b + std::sqrt(std::pow(b, 2) - 4 * a * c)) / 2;
    if (s < 0) s = std::numeric_limits<double>::quiet_NaN();
    laser_msgs.ranges.push_back(s);
    if (std::isnan(s)) continue;

    // validate data
    // solver validation
    Eigen::Vector3d point_calc;
    // point_calc[0] = s * std::cos(global_angle) + x;
    // point_calc[1] = s * std::sin(global_angle) + y;
    Eigen::Quaterniond lidar_rotation(q.w(), q.x(), q.y(), q.z());
    Eigen::Quaterniond ray_rotation(Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 0, 1)));
    point_calc = lidar_rotation.matrix() * ray_rotation.matrix() * Eigen::Vector3d(s * 1.0 , 0.0 , 0.0) + Eigen::Vector3d(x, y, 0.0);
    double range_calc = std::sqrt(point_calc[0] * point_calc[0] + point_calc[1] * point_calc[1]);
    double angle_calc = std::atan2(point_calc[1], point_calc[0]);

    if(!fequal(range_calc, FIXED_RANGE)) {
      std::cout << "calc range error" << idx << ": " << std::fabs(range_calc - FIXED_RANGE) << std::endl;
    }

    // doTransform validation
    Eigen::Vector3d point_in(std::cos(angle), std::sin(angle), 0.0);
    point_in *= s;
    Eigen::Vector3d point_out;

    tf2::doTransform(point_in, point_out, pose_intpl[idx]);
    double range = std::sqrt(point_out[0] * point_out[0] + point_out[1] * point_out[1]);
    double angle_out = std::atan2(point_out[1], point_out[0]);

    if (angle_out > M_PI) {
      angle_out -= 2 * M_PI;
    } else if (angle_out < - M_PI) {
      angle_out += 2 * M_PI;
    }

    if(!fequal(angle_out, angle_calc)) {
      std::cout << "dataset angle diff from transform" << idx << ": " << std::fabs(angle_out - angle_calc) << std::endl;
    }
    if(!fequal(range, FIXED_RANGE)) {
      std::cout << "dataset diff of point from transform" << idx << ": " << std::fabs(range - FIXED_RANGE) << std::endl;
    }
  }

  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  test_candidate_filter->update(laser_msgs, pointcloud_msg);

  tf2::doTransform(pointcloud_msg, pointcloud_msg, pose_intpl.back());
  // create undistorted pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pointcloud_msg, *point_cloud);

  // create distorted pointcloud from laser_msg
  pcl::PointCloud<pcl::PointXYZ> pcl_distort; // output pointcloud
  for (size_t idx = 0; idx < laser_msgs.ranges.size(); idx++) {
    if (std::isnan(laser_msgs.ranges[idx])) continue;
    Eigen::Vector3d point_in(
      std::cos(laser_msgs.angle_min + static_cast<float>(idx) * laser_msgs.angle_increment),
      std::sin(laser_msgs.angle_min + static_cast<float>(idx) * laser_msgs.angle_increment), 0.0);
    point_in *= laser_msgs.ranges[idx];
    pcl::PointXYZ pcl_pt;
    pcl_pt.x = point_in(0); pcl_pt.y = point_in(1); pcl_pt.z = point_in(2); // point conversion
    pcl_distort.push_back(pcl_pt);
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_distort_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  colorize(*point_cloud, *point_cloud_rgb, {125, 125, 255});
  colorize(pcl_distort, *pcl_distort_rgb, {255, 0, 0});

  // viewer option
  pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");

  viewer1.addPointCloud<pcl::PointXYZRGB>(point_cloud_rgb, "processed");
  viewer1.addPointCloud<pcl::PointXYZRGB>(pcl_distort_rgb, "original");

  viewer1.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "processed");
  viewer1.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original");

  // add robot motion
  double axis_size = 0.2;
  for (size_t idx = 0; idx < odom_set.size(); idx++) {
    Eigen::Affine3d pose = tf2::transformToEigen(odom_set[idx]);
    viewer1.addCoordinateSystem(axis_size, Eigen::Affine3f(pose));
    if (idx > 0) {
      Eigen::Affine3d pose_prev = tf2::transformToEigen(odom_set[idx - 1]);
      pcl::PointXYZ point;
      point.x = pose.translation().x();
      point.y = pose.translation().y();
      point.z = pose.translation().z();

      pcl::PointXYZ point_prev;
      point_prev.x = pose_prev.translation().x();
      point_prev.y = pose_prev.translation().y();
      point_prev.z = pose_prev.translation().z();
      std::string id = "arrowi" + std::to_string(idx);
      viewer1.addArrow<pcl::PointXYZ>(point, point_prev, 0.3, 0.3, 1.0, false, id);
    }
    // from world coordinate
    {
      Eigen::Affine3d pose_prev = tf2::transformToEigen(odom_set[idx - 1]);
      pcl::PointXYZ point;
      point.x = pose.translation().x();
      point.y = pose.translation().y();
      point.z = pose.translation().z();

      pcl::PointXYZ point_prev;
      point_prev.x = 0.0;
      point_prev.y = 0.0;
      point_prev.z = 0.0;
      std::string id = "arrowg" + std::to_string(idx);
      viewer1.addArrow<pcl::PointXYZ>(point, point_prev, 1.0, 0.3, 0.3, false, id);
    }
  }

  for (size_t idx = 0; idx < pose_intpl.size(); idx++) {
    Eigen::Affine3d pose = tf2::transformToEigen(pose_intpl[idx]);
    viewer1.addCoordinateSystem(0.3 * axis_size, Eigen::Affine3f(pose));
    if (idx > 0) {
      Eigen::Affine3d pose_prev = tf2::transformToEigen(pose_intpl[idx - 1]);
      pcl::PointXYZ point;
      point.x = pose.translation().x();
      point.y = pose.translation().y();
      point.z = pose.translation().z();

      pcl::PointXYZ point_prev;
      point_prev.x = pose_prev.translation().x();
      point_prev.y = pose_prev.translation().y();
      point_prev.z = pose_prev.translation().z();
      std::string id = "arrow" + std::to_string(idx);
      viewer1.addArrow<pcl::PointXYZ>(point, point_prev, 1.0, 1.0, 1.0, false, id);
    }
  }

  // add ground trouth
  pcl::ModelCoefficients circle_coeff;
  circle_coeff.values.resize (3);    // We need 3 values
  circle_coeff.values[0] = 0.0;
  circle_coeff.values[1] = 0.0;
  circle_coeff.values[2] = FIXED_RANGE;

  viewer1.addCircle(circle_coeff, "circle");
  viewer1.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, "circle");
  viewer1.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "circle");

  // set window color
  viewer1.setBackgroundColor(0.18039215686, 0.20392156862, 0.21176470588);
  while(!viewer1.wasStopped()) {
    viewer1.spinOnce();
  }

  size_t pc_size = point_cloud->size();
  bool failed = false;
  for (size_t i = 0; i < pc_size; i++) {
    float range = std::sqrt(std::pow((*point_cloud)[i].x, 2) + std::pow((*point_cloud)[i].y, 2));

    if(!fequal(range, FIXED_RANGE)) {
      failed = true;
      std::cout << "diff of point " << i << ": " << std::fabs(range - FIXED_RANGE) << std::endl;
    }
  }
  if (failed) std::cout << "\n[FAILED] improper range!" << std::endl;

  return 0;
}

