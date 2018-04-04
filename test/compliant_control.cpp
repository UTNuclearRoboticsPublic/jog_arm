
//#include "prettyprint/prettyprint.hpp"
#include <compliant_control.h>
#include <gtest/gtest.h>

using testing::Types;

namespace compliant_control_test {

TEST(CompliantControlTest, constructor) {
  std::vector<double> b(6, DBL_MAX);
  std::vector<double> condF(6, 0.0);
  std::vector<bool> endCondition(6, false);
  std::vector<double> haltF(6, 20.0);
  geometry_msgs::WrenchStamped ftData0;
  //  ROS_ERROR_STREAM(ftData0);
  compliant_control::CompliantControl control(b, condF, haltF, endCondition,
                                              ftData0);
}

TEST(CompliantControlTest, setDampingCoefficient) {
  std::vector<double> b(6, DBL_MAX);
  std::vector<double> condF(6, 0.0);
  std::vector<bool> endCondition(6, false);
  std::vector<double> haltF(6, 20.0);
  geometry_msgs::WrenchStamped ftData0;
  compliant_control::CompliantControl control(b, condF, haltF, endCondition,
                                              ftData0);

  std::vector<double> bin(6, DBL_MAX);
  bin[0] = 12.0;
  bin[1] = 10.0;
  bin[3] = 1e-4;
  bin[4] = -12.0;

  control.setDampingCoefficient(bin);

  EXPECT_NEAR(control.b_[0], 12.0, 1e-4);
  EXPECT_NEAR(control.b_[1], 10.0, 1e-4);
  EXPECT_NEAR(control.b_[2], DBL_MAX, 1e-4);
  EXPECT_NEAR(control.b_[3], DBL_MAX, 1e-4);
  EXPECT_NEAR(control.b_[4], -12.0, 1e-4);
  EXPECT_NEAR(control.b_[5], DBL_MAX, 1e-4);
}

TEST(CompliantControlTest, setEndCondition) {
  std::vector<double> b(6, DBL_MAX);
  std::vector<double> condF(6, 0.0);
  std::vector<double> haltF(6, 20.0);
  std::vector<bool> endCondition(6, false);
  geometry_msgs::WrenchStamped ftData0;
  compliant_control::CompliantControl control(b, condF, haltF, endCondition,
                                              ftData0);

  std::vector<double> fCond(6, 0.0);
  fCond[0] = 12.0;
  fCond[1] = 10.0;
  fCond[3] = 1e-4;
  fCond[4] = -12.0;

  control.setEndCondition(fCond);

  EXPECT_NEAR(control.condF_[0], 12.0, 1e-4);
  EXPECT_NEAR(control.condF_[1], 10.0, 1e-4);
  EXPECT_NEAR(control.condF_[2], 0.0, 1e-4);
  EXPECT_NEAR(control.condF_[3], 0.0, 1e-4);
  EXPECT_NEAR(control.condF_[4], 0.0, 1e-4);
  EXPECT_NEAR(control.condF_[5], 0.0, 1e-4);

  EXPECT_TRUE(control.endCondition_[0]);
  EXPECT_TRUE(control.endCondition_[1]);
  EXPECT_FALSE(control.endCondition_[2]);
  EXPECT_FALSE(control.endCondition_[3]);
  EXPECT_FALSE(control.endCondition_[4]);
  EXPECT_FALSE(control.endCondition_[5]);
}

TEST(CompliantControlTest, getVelocity) {
  std::vector<double> b(6, DBL_MAX);
  b[1] = 12.0;
  b[2] = 10.0;
  b[3] = 10.0;
  b[4] = -12.0;

  std::vector<double> condF(6, 0.0);
  condF[0] = 1e-4;
  condF[1] = 10.0;
  condF[2] = 12.0;
  condF[4] = -12.0;

  // None of the elements are controlled
  std::vector<bool> endCondition(6, false);
  std::vector<double> haltF(6, 20.0);

  geometry_msgs::WrenchStamped ftData0;
  compliant_control::CompliantControl control(b, condF, haltF, endCondition,
                                              ftData0);

  geometry_msgs::TwistStamped vIn, vOut;
  vIn.twist.linear.x = 1.0;
  vIn.twist.linear.y = 3.0;
  vIn.twist.linear.x = 10.0;
  vIn.twist.angular.x = 0.0;
  vIn.twist.angular.y = 1e-5;
  vIn.twist.angular.x = 0.9;

  geometry_msgs::WrenchStamped ftData;
  ftData.wrench.force.x = 10.0;
  ftData.wrench.force.y = 10.0;
  ftData.wrench.force.z = 10.0;
  ftData.wrench.torque.x = 10.0;
  ftData.wrench.torque.y = 10.0;
  ftData.wrench.torque.z = 10.0;

  Compliant_enum::EndCondition endcondition1 =
      control.getVelocity(vIn, ftData, vOut);
  EXPECT_TRUE(endcondition1 == Compliant_enum::NOT_CONTROLLED);
  EXPECT_EQ(vOut.twist.linear.x, vIn.twist.linear.x);
  EXPECT_EQ(vOut.twist.linear.y, vIn.twist.linear.y);
  EXPECT_EQ(vOut.twist.linear.z, vIn.twist.linear.z);
  EXPECT_EQ(vOut.twist.angular.x, vIn.twist.angular.x);
  EXPECT_EQ(vOut.twist.angular.y, vIn.twist.angular.y);
  EXPECT_EQ(vOut.twist.angular.z, vIn.twist.angular.z);

  // Two elements are controlled
  control.setEndCondition(condF);
  Compliant_enum::EndCondition endcondition2 =
      control.getVelocity(vIn, ftData, vOut);
  EXPECT_TRUE(endcondition2 == Compliant_enum::CONDITION_NOT_MET);
  EXPECT_EQ(vOut.twist.linear.x, vIn.twist.linear.x);
  EXPECT_EQ(vOut.twist.linear.y, vIn.twist.linear.y + 10.0 / b[1]);
  EXPECT_EQ(vOut.twist.linear.z, vIn.twist.linear.z + 10.0 / b[2]);
  EXPECT_EQ(vOut.twist.angular.x, vIn.twist.angular.x);
  EXPECT_EQ(vOut.twist.angular.y, vIn.twist.angular.y);
  EXPECT_EQ(vOut.twist.angular.z, vIn.twist.angular.z);

  //  To test std_msgs::String length
  char cmd[74];
  sprintf(cmd, "speedl([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 0.2, 0.1)\n",
          vOut.twist.linear.x, vOut.twist.linear.y, vOut.twist.linear.z,
          vOut.twist.angular.x, vOut.twist.angular.y, vOut.twist.angular.z);

  //  One of the controlled element has met the end condition and velocity in
  //  that direction should be zero.
  ftData.wrench.force.y = 14.0;

  Compliant_enum::EndCondition endcondition3 =
      control.getVelocity(vIn, ftData, vOut);
  EXPECT_TRUE(endcondition3 == Compliant_enum::CONDITION_MET);
  EXPECT_EQ(vOut.twist.linear.x, vIn.twist.linear.x);
  EXPECT_EQ(vOut.twist.linear.y, 0.0);
  EXPECT_EQ(vOut.twist.linear.z, vIn.twist.linear.z + 10.0 / b[2]);
  EXPECT_EQ(vOut.twist.angular.x, vIn.twist.angular.x);
  EXPECT_EQ(vOut.twist.angular.y, vIn.twist.angular.y);
  EXPECT_EQ(vOut.twist.angular.z, vIn.twist.angular.z);

  //  One of the controlled element has met the end condition and velocity in
  //  that direction should be zero.
  ftData.wrench.force.x = 24.0;

  Compliant_enum::EndCondition endcondition4 =
      control.getVelocity(vIn, ftData, vOut);
  EXPECT_TRUE(endcondition4 == Compliant_enum::FT_VIOLATION);
  EXPECT_EQ(vOut.twist.linear.x, 0.0);
  EXPECT_EQ(vOut.twist.linear.y, 0.0);
  EXPECT_EQ(vOut.twist.linear.z, 0.0);
  EXPECT_EQ(vOut.twist.angular.x, 0.0);
  EXPECT_EQ(vOut.twist.angular.y, 0.0);
  EXPECT_EQ(vOut.twist.angular.z, 0.0);
}
}
