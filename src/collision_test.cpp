#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thread>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Limit checker logic for verification
//Indicate the caller must use the result, otherwise complier error
[[nodiscard]] bool verifyTableSafety(double x_pose) {
  return x_pose >= 0.3;
}

// Cartesian path planning routine
[[nodiscard]] double planCartesianTrajectory(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  moveit_msgs::msg::RobotTrajectory& trajectory) 
{
  // Resolution check size: 1 cm step size for precision path interpolation
  const double eef_step = 0.01; 
  
  // Singularity safety limit: 0.0 disables checking, a small value restricts sudden joint jumps
  const double jump_threshold = 0.0; 

  // Compute the path. It returns the fraction of the path successfully calculated (0.0 to 1.0)
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  return fraction;
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
  node->declare_parameter<double>("retraction_height", 0.10);  // Motion Step 3, default moving distance 10 cm 
  node->declare_parameter<double>("home_x_pose", 0.3);         // Motion Step 3, Home Position X 
  node->declare_parameter<double>("home_y_pose", -0.2);        // Motion Step 3, Home Position Y
  node->declare_parameter<double>("home_z_pose", 0.5);         // Motion Step 3, Home Position Z

  // 3. Initialize a multi-threaded executor to handle parameter updates concurrently 
  //(Spinner thread), ensure the action/subscription executed
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  spinner.detach(); // Detach the thread to run independently in the background

  //4. Setup parameters 
  //i) Setup TF2 static background buffers inside CPU memory
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  //ii) Setup MoveGroupInterface (MUST match the SRDF group name: "ur_manipulator")
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "ur_manipulator");

  // Instantiate a PlanningSceneInterface object to interact with the environment
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //5. Fetch the parameters at runtime
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
  
  // 6.Setup Collision Object  
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
    table_pose.position.x = table_x; 
    table_pose.position.y = 0.0;
    table_pose.position.z = table_z;

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

   // Apply the object synchronously to the planning scene
    RCLCPP_INFO(node->get_logger(), "Table applied to scene at dynamic coordinates.");
   
    // Optional: Wait for 5 seconds to let RViz visually render the table and allow runtime param testing
    RCLCPP_INFO(node->get_logger(), "Waiting 5 seconds... You can run 'ros2 param set' now in another terminal!");
    rclcpp::sleep_for(std::chrono::seconds(5));
    
    // Verify X position 
    node->get_parameter("table_x_pose", table_x);
    RCLCPP_INFO(node->get_logger(), "Executing plan with final Table X: %f", table_x);

  // ===========================================================================
  // MOTION STEP 1: COMPUTE & EXECUTE CARTESIAN LINEAR PATH (Move STRAIGHT 10cm from Point A to Point B )
  // ===========================================================================
  std::vector<geometry_msgs::msg::Pose> linear_waypoints;
  
  // Step 1: Query and log current starting pose profile
  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
  linear_waypoints.push_back(current_pose);

  // Step 2: Define and append linear target displacement (FIXED: Resolved duplicate declaration variable mismatch)
  double retraction_height = 0.10;
  node->get_parameter("retraction_height", retraction_height);
  RCLCPP_INFO(node->get_logger(), "Using parametric retraction height: %f m", retraction_height);
   
  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.z += retraction_height; // Offset input straight up on the vertical Z-axis
  linear_waypoints.push_back(target_pose);

  // Step 3: Compute straight-line interpolation path
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  double fraction = planCartesianTrajectory(move_group, linear_waypoints, cartesian_trajectory);
  
  // Step 4: Validate completion percentage against safety thresholds before triggering actuators
  if (fraction >= 0.9) {
    RCLCPP_INFO(node->get_logger(), "Step 1: Executing Linear Retraction (A -> B)...");
    move_group.execute(cartesian_trajectory);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Linear path unsafe! Aborting sequence.");
    rclcpp::shutdown();
    return -1;
  }

  // Pause 
  rclcpp::sleep_for(std::chrono::milliseconds(500));
   
  // ===========================================================================
  // MOTION STEP 2: DYNAMIC TF2 TRACKING TRANSIT (Point B -> Dynamic Target Object C)
  // ===========================================================================
  move_group.setStartStateToCurrentState();

  // Create an explicit geometry message to receive the converted tracking values
  geometry_msgs::msg::Pose ptBtoptC_target_pose;
  bool tf_lookup_success = false;
  
  // Safety watchdog: attempt to parse the dynamic coordinate from the camera system
  try {
    // Look up the transform matrix from base_link to dummy vision tracking marker frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    
    RCLCPP_INFO(node->get_logger(), "TF2 Watchdog: Querying live position of 'dynamic_target_object'...");
    
    // Fetch latest coordinate within a strict 200ms timeout window
    transform_stamped = tf_buffer->lookupTransform(
      move_group.getPlanningFrame(), // Typically "base_link"
      "dynamic_target_object",       // The target frame moving dynamically in space
      tf2::TimePointZero,            // Fetch the absolute newest available message frame
      tf2::durationFromSec(0.2)      // 200 milliseconds timeout protection gate
    );

    // Map the stamped transform data in buffer directly into MoveIt target geometry structure
    ptBtoptC_target_pose.position.x = transform_stamped.transform.translation.x;
    ptBtoptC_target_pose.position.y = transform_stamped.transform.translation.y;
    ptBtoptC_target_pose.position.z = transform_stamped.transform.translation.z;
    ptBtoptC_target_pose.orientation = transform_stamped.transform.rotation;
    
    tf_lookup_success = true;
    RCLCPP_INFO(node->get_logger(), "TF2 Lock Secured! Target found at X:%f, Y:%f, Z:%f", 
                ptBtoptC_target_pose.position.x, ptBtoptC_target_pose.position.y, ptBtoptC_target_pose.position.z);
                
  } catch (const tf2::TransformException & ex) {
    // Safety Interlock: Trigger warning and block execution if the target frame is lost or dropped
    RCLCPP_ERROR(node->get_logger(), "VISION FAULT INTERLOCK TRIGGERED: Could not resolve target frame! Error: %s", ex.what());
  }

  // Actuator execution protection gate
  if (tf_lookup_success) {
    move_group.setPoseTarget(ptBtoptC_target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(success) {
      RCLCPP_INFO(node->get_logger(), "Motion Step 2: Planning success, executing Free-space transit to dynamic target object...");
      move_group.execute(my_plan);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Motion Step 2: Planning failed! Path to dynamic target object blocked.");
    }
  } else {
    RCLCPP_FATAL(node->get_logger(), "CRITICAL PIPELINE ABORT: Execution blocked due to missing transform vectors.");
  }

  // Pause 
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // ===========================================================================
  // MOTION STEP 3: RE-PLAN & EXECUTE FREE-SPACE PATH (Move from Point C to Point D)
  // ===========================================================================
    move_group.setStartStateToCurrentState();
  
  //Define Home Position
  double home_x = 0.3;
  double home_y = -0.2;
  double home_z = 0.5;
  node->get_parameter("home_x_pose", home_x);
  node->get_parameter("home_y_pose", home_y);
  node->get_parameter("home_z_pose", home_z);
  RCLCPP_INFO(node->get_logger(), "Get Home position: [%f, %f, %f]", home_x, home_y, home_z);

  // Define Target Position 2
  geometry_msgs::msg::Pose target_pose_2;
  target_pose_2.orientation.w = 1.0;
  target_pose_2.position.x = home_x_pose;
  target_pose_2.position.y = home_y_pose;
  target_pose_2.position.z = home_z_pose;
  move_group.setPoseTarget(target_pose_2);

 // Compute the Open Motion Planning Library plan dynamically *while standing at Point C*
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    RCLCPP_INFO(node->get_logger(), "Motion Step 3: Planning success, executing Free-space Transit (Point C -> Point D)...");
    move_group.execute(my_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Motion Step 3: Planning failed! Path blocked or unreachable.");
  }

  
  rclcpp::sleep_for(std::chrono::seconds(5));
  rclcpp::shutdown();
  return 0;
}