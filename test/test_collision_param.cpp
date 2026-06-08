#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>


// Limit checker logic for verification
//Indicate the caller must use the result, otherwise complier error
[[nodiscard]] bool verifyTableSafety(double x_pose) {
  return x_pose >= 0.3;
}


//(Test Case Name: ParamTestSuite, Purpose: InitialCheck)
//TEST(ParamTestSuite, InitialCheck) {
 //EXPECT_EQ(1, 2) << "TDD Setup Error: This test is designed to fail to prove the test runner works!";
//}

// Test Suite 01 - TEST UNIT 1: Validate system rejection of sub-boundary conditions
TEST(TS01TableSafety, RejectsDangerousCloseProximity) {
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto test_node = std::make_shared<rclcpp::Node>("test_dangerous_node", node_options);
  
  // Setup violating threshold (0.1m is inside the 0.3m forbidden zone)
  test_node->declare_parameter<double>("table_x_pose", 0.1);
  double dangerous_x;
  test_node->get_parameter("table_x_pose", dangerous_x);

  // Assert that validation routine must output false
  EXPECT_FALSE(verifyTableSafety(dangerous_x)) 
    << "Safety Interlock Failed: System accepted a table coordinate inside the forbidden radius!";
}

// Test Suite 01 - TEST UNIT 2: Validate system acceptance of clear nominal conditions
TEST(TS01TableSafety, AcceptsNominalSafeProximity) {
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto test_node = std::make_shared<rclcpp::Node>("test_safe_node", node_options);
  
  // Setup valid nominal operational parameter (0.6m)
  test_node->declare_parameter<double>("table_x_pose", 0.6);
  double safe_x;
  test_node->get_parameter("table_x_pose", safe_x);

  // Assert that validation routine must output true
  EXPECT_TRUE(verifyTableSafety(safe_x))
    << "Operational Failure: System rejected a perfectly safe nominal table layout!";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}