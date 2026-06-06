#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

//(Test Case Name: ParamTestSuite, Purpose: InitialCheck)
TEST(ParamTestSuite, InitialCheck) {
 EXPECT_EQ(1, 2) << "TDD Setup Error: This test is designed to fail to prove the test runner works!";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}