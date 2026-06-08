#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

bool verifyTableSafety(double x_pose) {
  // When X parameters is out of range then false
  if (x_pose < 0.3){
  return false; 
  }else{
  return true;
  } 
}

//(Test Case Name: ParamTestSuite, Purpose: InitialCheck)
//TEST(ParamTestSuite, InitialCheck) {
 //EXPECT_EQ(1, 2) << "TDD Setup Error: This test is designed to fail to prove the test runner works!";
//}

TEST(ParamTestSuite, DangerousTableXParamCheck) {
  // 1. Conifgure ROS 2 Node Options
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  // 2. Test  Node
  auto test_node = std::make_shared<rclcpp::Node>("test_param_node", node_options);
  
  // 3. Simulate false parameters input
  test_node->declare_parameter<double>("table_x_pose", 0.1);
  
  // 4. Read this parameter during execution
  double current_table_x = 0.6;
  test_node->get_parameter("table_x_pose", current_table_x);
  
  // 5. Test result, a fail test case with verifyTableSafety is always true
  EXPECT_FALSE(verifyTableSafety(current_table_x)) 
    << "CRITICAL SAFETY FAILURE: The system accepted a dangerous table position (< 0.3m)!";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}