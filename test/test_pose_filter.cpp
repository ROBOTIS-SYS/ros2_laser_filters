#include <ros2_laser_filters/pose_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

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

  x_fix = 3.12351423617;
  y_fix = 4.1462345624578;
  z_fix = 5.123416146;
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
    pose_increment.translate(Eigen::Vector3d(2 * pt_plus, 0, - pt_plus));
    auto axis = Eigen::Vector3d(test_no/test_size, 1, 1);
    axis.normalize();
    pose_increment.rotate(Eigen::AngleAxisd(0.628, axis));

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

  return 0;
}

