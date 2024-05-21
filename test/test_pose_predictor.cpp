#include <ros2_laser_filters/pose_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

int main (int argc, char * argv[]) {

  std::cout << "\n[INFO] =========== start test of class PosePredictorBase =========== \n"
    << std::endl;

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Time ref_time = node->get_clock()->now();
  std::cout << "[INFO] test starting time : " << static_cast<int>(ref_time.seconds()) << std::endl;
  // create interpolation fixture
  std::vector<geometry_msgs::msg::TransformStamped> odom_set;


  /** test parameters **/
  std::string frame_id = "odom";
  std::string child_frame_id = "base_link";

  size_t test_size = 100;
  double rate_max = 1.0;
  double rate_min = 0.0;

  double due_min_s = 9.0;
  double due_max_s = -1.0;
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



  std::unique_ptr<laser_filters::PosePredictorBase> predictor;
  predictor = std::make_unique<laser_filters::PosePredictorBase>();

  // {
  //   std::cout << "[INFO] timestamps in odom_set : [ ";
  //   for (auto it = odom_set.begin(); it != odom_set.end(); it++) {
  //     std::cout << it->header.stamp.sec << ", ";
  //   }
  //   std::cout << "]\n";
  // }

  // {
  //   std::cout << "[INFO] pose in odom_set : " << std::endl;
  //   for (auto it = odom_set.begin(); it != odom_set.end(); it++) {
  //     std::cout << "[" << it->transform.translation.x << ", " <<
  //       it->transform.translation.y << ", " <<
  //       it->transform.translation.z << "]\n";
  //   }
  // }


  // create scan time stamp
  rclcpp::Time scan_time = rclcpp::Time(odom_set[std::floor(test_size/2)].header.stamp) +
    rclcpp::Duration::from_seconds(DUE_OFFSET);
  std::cout << "\n[INFO] =========== start test add_pose(), pop_buffer() =========== \n"
    << std::endl;

  /** test start **/
  // input test
  for (size_t i = 0; i < test_size; i++){
    predictor->add_pose(odom_set[i]);
    auto pose_buff = predictor->get_pose_buffer();

    // std::cout << "[INFO] current predictor.pose_buffer status : [ ";
    // for (auto it = pose_buff.begin(); it != pose_buff.end(); it++) {
    //   std::cout << it->header.stamp.sec << ", ";
    // }
    // std::cout << "]\n";

  }

  /*** add_pose() : report analysis ***/

  // sorting test
  bool is_error = false;
  auto pose_sort = predictor->get_pose_buffer();
  for (auto it = pose_sort.begin(); it != pose_sort.end() - 1; it++ ) {
    if (rclcpp::Time(it->header.stamp) > (it+1)->header.stamp) {
      std::cout << "[FAILED] pop_buffer(): unordered timestamp" << std::endl;
      is_error = true;
    }
  }
  if (!is_error) {
    std::cout << "[SUCCESS] pop_buffer(): properly ordered timestamp" << std::endl;
  }

  std::cout << "[INFO] current pose_sort data in time " << static_cast<int>(scan_time.seconds()) << " : [ ";
  for (auto it = pose_sort.begin(); it != pose_sort.end(); it++) {
    std::cout << it->header.stamp.sec << ", ";
  }
  std::cout << "]\n";

  int num_div = 5;
  std::vector<rclcpp::Time> scan_dev_stamps;
  std::vector<double> dev_rates;

  // insert tail (before odom time stamp)

  auto tail_due = rclcpp::Time(pose_sort[1].header.stamp) - pose_sort[0].header.stamp;

  for (int div = num_div - 1; div >= 0; div--) {
    double rate = - div / num_div;
    scan_dev_stamps.push_back(rclcpp::Time(pose_sort.front().header.stamp) + tail_due * rate);
    dev_rates.push_back(rate);
  }

  for (auto it = pose_sort.begin(); it != pose_sort.end() - 1; it++) {
    auto div_due = rclcpp::Time((it + 1)->header.stamp) - it->header.stamp;

    for (int div = 0; div < num_div; div++) {
      double rate = div / num_div;

      scan_dev_stamps.push_back(rclcpp::Time(it->header.stamp) + div_due * rate);
      dev_rates.push_back(rate);
    }
  }

  // insert head(after odom time stamp)
  auto head_due = rclcpp::Time(pose_sort.back().header.stamp) - pose_sort[pose_sort.size() - 2].header.stamp;

  for (int div = 0; div < num_div; div++) {
    double rate = div / num_div;
    scan_dev_stamps.push_back(rclcpp::Time(pose_sort.back().header.stamp) + head_due * rate);
    dev_rates.push_back(rate + 1.0);
  }

  std::vector<geometry_msgs::msg::TransformStamped> pose_intpl;
  predictor->interpolate_poses(scan_dev_stamps, pose_intpl);

  is_error = false;

  // success evaluation
  for (size_t idx = 0; idx < scan_dev_stamps.size(); idx++) {
    auto it = pose_sort.begin();
    for (; it != pose_sort.end(); it++) {
      if (rclcpp::Time(pose_intpl[idx].header.stamp) < it->header.stamp) { break; }
    }
    if (it == pose_sort.end()) {
      it--;
    }
    geometry_msgs::msg::Vector3 pt1 = (it - 1)->transform.translation;
    geometry_msgs::msg::Vector3 pt2 = it->transform.translation;

    double ex = (pt2.x - pt1.x) * dev_rates[idx] -
      (pose_intpl[idx].transform.translation.x - pt1.x);
    double ey = (pt2.y - pt1.y) * dev_rates[idx] -
      (pose_intpl[idx].transform.translation.y - pt1.y);
    double ez = (pt2.z - pt1.z) * dev_rates[idx] -
      (pose_intpl[idx].transform.translation.z - pt1.z);

    is_error = false;
    bool pose_error = false;
    if (ex * ex > 0.001) {
      pose_error = true;
      std::cout << "[FAILED] ex in idx" << idx << " have large error of " << ex << std::endl;
    }
    if (ey * ey > 0.001) {
      pose_error = true;
      std::cout << "[FAILED] ey in idx" << idx << " have large error of " << ey << std::endl;
    }
    if (ez * ez > 0.001) {
      pose_error = true;
      std::cout << "[FAILED] ez in idx" << idx << " have large error of " << ez << std::endl;
    }
    if (pose_error) {
      is_error = true;

      std::cout << "[INFO] pre-interplation value: " << pt1.x  << ", " <<
        pt1.y << ", " <<
        pt1.z << std::endl;

      std::cout << "[INFO] post-interplation value: " << pt2.x  << ", " <<
        pt2.y << ", " <<
        pt2.z << std::endl;

      std::cout << "[INFO] location value : " << pose_intpl[idx].transform.translation.x << ", " <<
        pose_intpl[idx].transform.translation.y << ", " <<
        pose_intpl[idx].transform.translation.z << std::endl;

      std::cout << "[INFO] error value: " << ex << ", " << ey << ", " << ez << std::endl;
    }
  }
  if (is_error) {
    std::cout << "[FAILED] interpolate_poses(): proper translation interpolation"
        << std::endl;
  } else {
    std::cout << "[SUCCESS] interpolate_poses(): proper translation interpolation"
      << std::endl;
  }


  std::cout << "\n[INFO] ========== start testing backpropagate_pose() =========== \n" << std::endl;

  // create fixture

  std::vector<geometry_msgs::msg::TransformStamped> pose_backprop;
  predictor->backpropagate_pose(pose_intpl, pose_backprop);

  std::vector<geometry_msgs::msg::Pose> eval_poses;
  Eigen::Isometry3d back_pose;
  geometry_msgs::msg::Pose back_pose_msg;
  back_pose_msg.orientation = pose_intpl.back().transform.rotation;
  back_pose_msg.position.x = pose_intpl.back().transform.translation.x;
  back_pose_msg.position.y = pose_intpl.back().transform.translation.y;
  back_pose_msg.position.z = pose_intpl.back().transform.translation.z;
  tf2::fromMsg(back_pose_msg, back_pose);

  std::cout << "pose_backprop size: " << pose_backprop.size() << std::endl;
  for (size_t idx = 0; idx < pose_backprop.size(); idx++) {
    Eigen::Isometry3d prop_pose;
    geometry_msgs::msg::Pose conversion_pose;
    conversion_pose.orientation = pose_backprop[idx].transform.rotation;
    conversion_pose.position.x = pose_backprop[idx].transform.translation.x;
    conversion_pose.position.y = pose_backprop[idx].transform.translation.y;
    conversion_pose.position.z = pose_backprop[idx].transform.translation.z;
    tf2::fromMsg(conversion_pose, prop_pose);
    geometry_msgs::msg::Pose eval_pose = tf2::toMsg(back_pose * prop_pose);
    eval_poses.push_back(eval_pose);
  }

  is_error = false;
  for (size_t idx = 0; idx < eval_poses.size(); idx++) {
    auto p = pose_intpl[idx];
    auto e = eval_poses[idx];

    double errors[3];
    errors[0] = p.transform.translation.x - e.position.x;
    errors[1] = p.transform.translation.y - e.position.y;
    errors[2] = p.transform.translation.z - e.position.z;
    errors[3] = p.transform.rotation.x - e.orientation.x;
    errors[4] = p.transform.rotation.y - e.orientation.y;
    errors[5] = p.transform.rotation.z - e.orientation.z;
    errors[6] = p.transform.rotation.w - e.orientation.w;

    double error = 0;
    for(int i = 0; i < 7; i++) {
      error += (errors[i] * errors[i]);
    }
    if (error > 0.000000001) {
      std::cout << "error :" << error << std::endl;
      is_error = true;
    }
  }

  if (is_error) {
    std::cout << "[FAILED] backpropagate_pose(): wrong position value"
        << std::endl;
  } else {
    std::cout << "[SUCCESS] backpropagate_pose(): proper position value"
        << std::endl;
  }

  // for (size_t idx = 0; idx < eval_poses.size(); idx++) {
  //   auto p = eval_poses[idx];
  //   std::cout << "xyz : [" << p.position.x << ", " << p.position.y << ", " << p.position.z << "]\n";
  // }
  // std::cout << "[INFO]: interpolated position" << std::endl;
  // for (size_t idx = 0; idx < eval_poses.size(); idx++) {
  //   auto p = pose_intpl[idx].transform.translation;
  //   std::cout << "xyz : [" << p.x << ", " << p.y << ", " << p.z << "]\n";
  // }

  std::cout << "\n[INFO] ========== start testing pipeline sustainability =========== \n" << std::endl;

  // flush out remaining data
  rclcpp::Time back_time = rclcpp::Time(predictor->get_pose_buffer().back().header.stamp) +
    rclcpp::Duration::from_seconds(1.0);

  predictor->pop_buffer(back_time);

  if (!predictor->can_predict()) {
    std::cout << "\n[FAILED] buffer size is less than 2 \n" << std::endl;
  }

  std::cout << "predictor size : " << predictor->get_pose_buffer().size() << std::endl;

  return 0;
}
