

#include "robot_localization/ros_filter_types.h"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

using RobotLocalization::Ukf;
using RobotLocalization::RosUkf;
using RobotLocalization::STATE_SIZE;

class RosUkfPassThrough : public RosUkf
{
 public:
  explicit RosUkfPassThrough(std::vector<double> &args) :
    RosUkf(ros::NodeHandle( ), ros::NodeHandle("~"), args)
  {
  }

  Ukf &getFilter( )
  {
    return filter_;
  }
};

TEST(UkfTest, Measurements)
{
  std::vector<double> args;
  args.push_back(0.001);
  args.push_back(0);
  args.push_back(2);

  RosUkfPassThrough ukf(args);

  Eigen::MatrixXd initialCovar(15, 15);
  initialCovar.setIdentity( );
  initialCovar *= 0.5;
  ukf.getFilter( ).setEstimateErrorCovariance(initialCovar);

  EXPECT_EQ(ukf.getFilter( ).getEstimateErrorCovariance( ), initialCovar);

  Eigen::VectorXd measurement(STATE_SIZE);
  for (size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurement[i] = i * 0.01 * STATE_SIZE;
  }

  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  measurementCovariance.setIdentity( );
  for (size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurementCovariance(i, i) = 1e-9;
  }

  std::vector<int> updateVector(STATE_SIZE, true);

  // Ensure that measurements are being placed in the queue correctly
  ros::Time time;
  time.fromSec(1000);
  ukf.enqueueMeasurement("odom0",
                         measurement,
                         measurementCovariance,
                         updateVector,
                         std::numeric_limits<double>::max( ),
                         time);

  ukf.integrateMeasurements(ros::Time(1001));

  EXPECT_EQ(ukf.getFilter( ).getState( ), measurement);
  EXPECT_EQ(ukf.getFilter( ).getEstimateErrorCovariance( ), measurementCovariance);

  ukf.getFilter( ).setEstimateErrorCovariance(initialCovar);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  for (size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurementCovariance(i, i) = 1e-9;
  }

  time.fromSec(1002);
  ukf.enqueueMeasurement("odom0",
                         measurement2,
                         measurementCovariance,
                         updateVector,
                         std::numeric_limits<double>::max( ),
                         time);

  ukf.integrateMeasurements(ros::Time(1003));

  measurement = measurement2.eval( ) - ukf.getFilter( ).getState( );
  for (size_t i = 0; i < STATE_SIZE; ++i)
  {
    EXPECT_LT(::fabs(measurement[i]), 0.001);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ukf");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS( );
}
