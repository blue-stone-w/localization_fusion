

#include "robot_localization/ros_robot_localization_listener.h"
#include "robot_localization/filter_common.h"

#include <string>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <gtest/gtest.h>

RobotLocalization::RosRobotLocalizationListener *g_listener;

TEST(LocalizationListenerTest, testGetStateOfBaseLink)
{
  ros::spinOnce( );

  ros::Time time2(1001);

  Eigen::VectorXd state(RobotLocalization::STATE_SIZE);
  Eigen::MatrixXd covariance(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE);

  std::string base_frame("base_link");
  g_listener->getState(time2, base_frame, state, covariance);

  EXPECT_DOUBLE_EQ(1.0, state(RobotLocalization::StateMemberX));
  EXPECT_DOUBLE_EQ(0.0, state(RobotLocalization::StateMemberY));
  EXPECT_DOUBLE_EQ(0.0, state(RobotLocalization::StateMemberZ));

  EXPECT_FLOAT_EQ(M_PI / 4, state(RobotLocalization::StateMemberRoll));
  EXPECT_FLOAT_EQ(0.0, state(RobotLocalization::StateMemberPitch));
  EXPECT_FLOAT_EQ(0.0, state(RobotLocalization::StateMemberYaw));

  EXPECT_DOUBLE_EQ(M_PI / 4.0, state(RobotLocalization::StateMemberVroll));
  EXPECT_DOUBLE_EQ(0.0, state(RobotLocalization::StateMemberVpitch));
  EXPECT_DOUBLE_EQ(0.0, state(RobotLocalization::StateMemberVyaw));
}

TEST(LocalizationListenerTest, GetStateOfRelatedFrame)
{
  ros::spinOnce( );

  Eigen::VectorXd state(RobotLocalization::STATE_SIZE);
  Eigen::MatrixXd covariance(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE);

  ros::Time time1(1000);
  ros::Time time2(1001);

  std::string sensor_frame("sensor");

  EXPECT_TRUE(g_listener->getState(time1, sensor_frame, state, covariance));

  EXPECT_FLOAT_EQ(0.0, state(RobotLocalization::StateMemberX));
  EXPECT_FLOAT_EQ(1.0, state(RobotLocalization::StateMemberY));
  EXPECT_FLOAT_EQ(0.0, state(RobotLocalization::StateMemberZ));

  EXPECT_FLOAT_EQ(0.0, state(RobotLocalization::StateMemberRoll));
  EXPECT_FLOAT_EQ(0.0, state(RobotLocalization::StateMemberPitch));
  EXPECT_FLOAT_EQ(M_PI / 2, state(RobotLocalization::StateMemberYaw));

  EXPECT_TRUE(1e-12 > state(RobotLocalization::StateMemberVx));
  EXPECT_FLOAT_EQ(-1.0, state(RobotLocalization::StateMemberVy));
  EXPECT_FLOAT_EQ(M_PI / 4.0, state(RobotLocalization::StateMemberVz));

  EXPECT_TRUE(1e-12 > state(RobotLocalization::StateMemberVroll));
  EXPECT_FLOAT_EQ(-M_PI / 4.0, state(RobotLocalization::StateMemberVpitch));
  EXPECT_FLOAT_EQ(0.0, state(RobotLocalization::StateMemberVyaw));

  EXPECT_TRUE(g_listener->getState(time2, sensor_frame, state, covariance));

  EXPECT_FLOAT_EQ(1.0, state(RobotLocalization::StateMemberX));
  EXPECT_FLOAT_EQ(sqrt(2) / 2.0, state(RobotLocalization::StateMemberY));
  EXPECT_FLOAT_EQ(sqrt(2) / 2.0, state(RobotLocalization::StateMemberZ));

  EXPECT_TRUE(1e-12 > state(RobotLocalization::StateMemberRoll));
  EXPECT_TRUE(1e-12 > fabs(-M_PI / 4.0 - state(RobotLocalization::StateMemberPitch)));
  EXPECT_FLOAT_EQ(M_PI / 2, state(RobotLocalization::StateMemberYaw));

  EXPECT_TRUE(1e-12 > state(RobotLocalization::StateMemberVx));
  EXPECT_FLOAT_EQ(-1.0, state(RobotLocalization::StateMemberVy));
  EXPECT_FLOAT_EQ(M_PI / 4, state(RobotLocalization::StateMemberVz));

  EXPECT_TRUE(1e-12 > state(RobotLocalization::StateMemberVroll));
  EXPECT_FLOAT_EQ(-M_PI / 4.0, state(RobotLocalization::StateMemberVpitch));
  EXPECT_FLOAT_EQ(0, state(RobotLocalization::StateMemberVyaw));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_robot_localization_estimator");

  g_listener = new RobotLocalization::RosRobotLocalizationListener( );

  testing::InitGoogleTest(&argc, argv);

  int res = RUN_ALL_TESTS( );

  delete g_listener;

  return res;
}
