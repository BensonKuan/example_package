#include <example_class.h>

//#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "ros/ros.h"

namespace {

class ExampleClassTest : public ::testing::Test {
 protected:
  ExampleClassTest()
      : example_int_(5) {
    example_class_ = std::make_unique<ExampleClass>();
  }

  TestSomething();

  int example_int_;
  std::unique_ptr<ExampleClass> example_class_;
};

TEST_F(ExampleClassTest, ExampleFixedTest) {
  example_class_->DoSomething();
  TestSomething();
}

TEST(TimeConversion, testToRos) {

  double seconds_since_epoch = 10;
  ::ros::Time ros_now;
  ros_now.fromSec(seconds_since_epoch);
  double seconds_since_epoch_after = 20;
  ::ros::Time ros_after;
  ros_after.fromSec(seconds_since_epoch_after);

  EXPECT_EQ(ros_now, ros_after);
}

} // namespace
