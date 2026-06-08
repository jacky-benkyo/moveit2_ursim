#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thread>
#include <vector>


// Limit checker logic for verification
//Indicate the caller must use the result, otherwise complier error
[[nodiscard]] bool verifyTableSafety(double x_pose) {
  return x_pose >= 0.3;
}

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  // 1. Set Node Options (automatically load the robot description from the Launch file)
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("ur_collision_check", node_options);
  
  //2. Declare ROS2 parameters
  node->declare_parameter<double>("table_x_pose", 0.6);
  node->declare_parameter<double>("table_z_pose", -0.05);

  // 3. Initialize a multi-threaded executor to handle parameter updates concurrently 
  //(Spinner thread), enusre the action/suscribtion executed
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  spinner.detach(); // Detach the thread to run independently in the background

  //setup MoveGroupInterface (MUST match the SRDF group name: "ur_manipulator")
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "ur_manipulator");

    // Instantiate a PlanningSceneInterface object to interact with the environment
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //4. Fetch the paramters at runtime
    double table_x = 0.6;
    double table_z = -0.05;
    node->get_parameter("table_x_pose", table_x);
    node->get_parameter("table_z_pose", table_z);

    // Runtime safety verification block (Safety Interlock)
    if (!verifyTableSafety(table_x)) {
    RCLCPP_FATAL(node->get_logger(), "EMERGENCY INTERLOCK TRIGGERED: Parameter 'table_x_pose' (%f) is too close to robot base! Aborting setup.", table_x);
    rclcpp::shutdown();
    return -1; 
    }
    RCLCPP_INFO(node->get_logger(), "Safety check passed. Table X: %f", table_x);
    RCLCPP_INFO(node->get_logger(), "Loaded parameter - Table X: %f, Table Z: %f", table_x, table_z);

  // Define a CollisionObject message
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "table";

    // Define the shape of the object as a solid Box
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5; // Length (x-axis)
    primitive.dimensions[1] = 1.0; // Width (y-axis)
    primitive.dimensions[2] = 0.1; // Thickness/Height (z-axis)

    // Define the pose (position and orientation) of the table relative to the frame_id
    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = table_x; // 0.4 b4
    table_pose.position.y = 0.0;
    table_pose.position.z = table_z; // 0.2 b4

    // Add the shape and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;
    
    // Push the collision object into a vector and apply it to the planning scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    collision_object.operation = collision_object.ADD;

    // Optional: Wait for 2 second to let RViz visually render the table before the robot moves
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Apply the object synchronously (blocks until MoveIt confirms it has received the object)
    planning_scene_interface.applyCollisionObject(collision_object);

    // Apply the object directly
         // Apply the object synchronously to the planning scene
    RCLCPP_INFO(node->get_logger(), "Table applied to scene at dynamic coordinates.");

    RCLCPP_INFO(node->get_logger(), "Waiting 5 seconds... You can run 'ros2 param set' now in another terminal!");
    rclcpp::sleep_for(std::chrono::seconds(5));
    
    // Verify X position 
    node->get_parameter("table_x_pose", table_x);
    RCLCPP_INFO(node->get_logger(), "Executing plan with final Table X: %f", table_x);

  //log info
  //RCLCPP_INFO(node->get_logger(), "Table added. Initializing motion planning for ur_manipulator");
  
  // define Pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.3;
  target_pose.position.y = -0.2;
  target_pose.position.z = 0.5;
  move_group.setPoseTarget(target_pose);

  // path planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    RCLCPP_INFO(node->get_logger(), "Planning success, executing motion ...");
    move_group.execute(my_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed! Path blocked or unreachable.");
  }

  
  rclcpp::sleep_for(std::chrono::seconds(5));
  rclcpp::shutdown();
  return 0;
}