#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Simulates the behavior of an incomplete Cartesian planner.
// Map function mapping directly to production implementation signature
[[nodiscard]] double planCartesianTrajectory(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  moveit_msgs::msg::RobotTrajectory& trajectory) 
{
  const double eef_step = 0.01;
  const double jump_threshold = 0.0;
  return move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
}

// Dedicated Integration Test Suite
// Test Suite 01 - TEST UNIT 1: Verify the loader YAML Parameters
TEST(TS01ParametersCheck, VerifyParameterLoading) {
  // 1. Locate the path of the installation share folder for the config/motion_params.yaml
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("my_moveit_app");
  std::string yaml_path = pkg_share_dir + "/config/motion_params.yaml";

  // 2. Inject the parameter file path into the ROS 2 node setup configuration
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.arguments({"--ros-args", "--params-file", yaml_path});

  // 3. Create a standalone isolated test node dedicated to check the parsing parameters
  auto test_param_node = std::make_shared<rclcpp::Node>("ur_collision_check", node_options);

  double retraction_height = 0.0;
  double home_x_pose = 0.0;
  double home_y_pose = 0.0;
  double home_z_pose = 0.0;

  // 4. Extract parameters from the loaded server storage pool
  bool check_retraction = test_param_node->get_parameter("retraction_height", retraction_height);
  bool check_home_x = test_param_node->get_parameter("home_x_pose", home_x_pose);
  bool check_home_y = test_param_node->get_parameter("home_y_pose", home_y_pose);
  bool check_home_z = test_param_node->get_parameter("home_z_pose", home_z_pose);

  // Assert A: Confirm parameter keys exist within the parsed configuration space
  ASSERT_TRUE(check_retraction) << "PARAM ERROR: 'retraction_height' was not discovered on the node parameter server!";
  ASSERT_TRUE(check_home_x) << "PARAM ERROR: 'home_x_pose' was not discovered on the node parameter server!";
  ASSERT_TRUE(check_home_y) << "PARAM ERROR: 'home_y_pose' was not discovered on the node parameter server!";
  ASSERT_TRUE(check_home_z) << "PARAM ERROR: 'home_z_pose' was not discovered on the node parameter server!";

  // Assert B: Cross-verify values precisely (0.00001.) match the metrics specified in motion_params.yaml 
  EXPECT_NEAR(retraction_height, 0.15, 1e-5) << "VALUE MISMATCH: 'retraction_height' loaded but the value is incorrect!";
  EXPECT_NEAR(home_x_pose, 0.4, 1e-5) << "VALUE MISMATCH: 'home_x_pose' loaded but the value is incorrect!";
  EXPECT_NEAR(home_y_pose, -0.3, 1e-5) << "VALUE MISMATCH: 'home_y_pose' loaded but the value is incorrect!";
  EXPECT_NEAR(home_z_pose, 0.6, 1e-5) << "VALUE MISMATCH: 'home_z_pose' loaded but the value is incorrect!";
}

// Test Suite 02 - TEST UNIT 2: Cartesian Path Tracking Precision Validation
TEST(TS02CartesianMotion, DemandsHighTrajectorySuccessRate) {
  // 1. Define a targeted linear waypoint path array
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto test_node = std::make_shared<rclcpp::Node>("test_cartesian_node", node_options);

  // Initialize background spinning executor to fetch active robot state metrics
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_node);
  std::thread spinner([&executor]() { executor.spin(); });
  spinner.detach();

  // Create real MoveGroup handle (Requires mock or simulator driver active in background)
  moveit::planning_interface::MoveGroupInterface move_group(test_node, "ur_manipulator");
 
  // Define a targeted linear waypoint path array starting from the current robot posture
  std::vector<geometry_msgs::msg::Pose> target_waypoints;
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
  target_waypoints.push_back(start_pose);
  
  // Linear offset segment: Move 10cm straight up on the Z-axis
  geometry_msgs::msg::Pose next_pose = start_pose;
  next_pose.position.z += 0.1; 
  target_waypoints.push_back(next_pose);
  
  moveit_msgs::msg::RobotTrajectory trajectory;
  double execution_fraction = planCartesianTrajectory(move_group, target_waypoints, trajectory);
  
  // Assert that the kinematic solver can safely compute a smooth linear path 
  // Expect fraction >= 90%
  EXPECT_GE(execution_fraction, 0.9) 
    << "SURGICAL PATHWAY ERROR: Linear interpolation failure. Trajectory completion is below safe threshold!";
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}