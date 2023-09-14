

#ifndef ROBOT_LOCALIZATION_ROS_ROBOT_LOCALIZATION_LISTENER_H
#define ROBOT_LOCALIZATION_ROS_ROBOT_LOCALIZATION_LISTENER_H

#include "robot_localization/robot_localization_estimator.h"

#include <string>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>

namespace RobotLocalization
{

//! @brief RosRobotLocalizationListener class
//!
//! This class wraps the RobotLocalizationEstimator. It listens to topics over which the (filtered) robot state is
//! published (odom and accel) and pushes them into its instance of the RobotLocalizationEstimator. It exposes a
//! getState method to offer the user the estimated state at a requested time. This class offers the option to run this
//! listener without the need to run a separate node. If you do wish to run this functionality in a separate node,
//! consider the robot localization listener node.
//!
class RosRobotLocalizationListener
{
 public:
  //! @brief Constructor
  //!
  //! The RosRobotLocalizationListener constructor initializes nodehandles, subscribers, a filter for synchronized
  //! listening to the topics it subscribes to, and an instance of the RobotLocalizationEstimator.
  //!
  RosRobotLocalizationListener( );

  //! @brief Destructor
  //!
  //! Empty destructor
  //!
  ~RosRobotLocalizationListener( );

  //! @brief Get a state from the localization estimator
  //!
  //! Requests the predicted state and covariance at a given time from the Robot Localization Estimator.
  //!
  //! @param[in] time - time of the requested state
  //! @param[in] frame_id - frame id of which the state is requested.
  //! @param[out] state - state at the requested time
  //! @param[out] covariance - covariance at the requested time
  //!
  //! @return false if buffer is empty, true otherwise
  //!
  bool getState(const double time, const std::string &frame_id,
                Eigen::VectorXd &state, Eigen::MatrixXd &covariance,
                std::string world_frame_id = "") const;

  //! @brief Get a state from the localization estimator
  //!
  //! Overload of getState method for using ros::Time.
  //!
  //! @param[in] ros_time - ros time of the requested state
  //! @param[in] frame_id - frame id of which the state is requested.
  //! @param[out] state - state at the requested time
  //! @param[out] covariance - covariance at the requested time
  //!
  //! @return false if buffer is empty, true otherwise
  //!
  bool getState(const ros::Time &ros_time, const std::string &frame_id,
                Eigen::VectorXd &state, Eigen::MatrixXd &covariance,
                const std::string &world_frame_id = "") const;

  //!
  //! \brief getBaseFrameId Returns the base frame id of the localization listener
  //! \return The base frame id
  //!
  const std::string &getBaseFrameId( ) const;

  //!
  //! \brief getWorldFrameId Returns the world frame id of the localization listener
  //! \return The world frame id
  //!
  const std::string &getWorldFrameId( ) const;

 private:
  //! @brief Callback for odom and accel
  //!
  //! Puts the information from the odom and accel messages in a Robot Localization Estimator state and sets the most
  //! recent state of the estimator.
  //!
  //! @param[in] odometry message
  //! @param[in] accel message
  //!
  void odomAndAccelCallback(const nav_msgs::Odometry &odom, const geometry_msgs::AccelWithCovarianceStamped &accel);

  //! @brief The core state estimator that facilitates inter- and
  //! extrapolation between buffered states.
  //!
  RobotLocalizationEstimator *estimator_;

  //! @brief A public handle to the ROS node
  //!
  ros::NodeHandle nh_;

  //! @brief A private handle to the ROS node
  //!
  ros::NodeHandle nh_p_;

  //! @brief Subscriber to the odometry state topic (output of a filter node)
  //!
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

  //! @brief Subscriber to the acceleration state topic (output of a filter node)
  //!
  message_filters::Subscriber<geometry_msgs::AccelWithCovarianceStamped> accel_sub_;

  //! @brief Waits for both an Odometry and an Accel message before calling a single callback function
  //!
  message_filters::TimeSynchronizer<nav_msgs::Odometry, geometry_msgs::AccelWithCovarianceStamped> sync_;

  //! @brief Child frame id received from the Odometry message
  //!
  std::string base_frame_id_;

  //! @brief Frame id received from the odometry message
  //!
  std::string world_frame_id_;

  //! @brief Tf buffer for looking up transforms
  //!
  tf2_ros::Buffer tf_buffer_;

  //! @brief Transform listener to fill the tf_buffer
  //!
  tf2_ros::TransformListener tf_listener_;
};

} // namespace RobotLocalization

#endif // ROBOT_LOCALIZATION_ROS_ROBOT_LOCALIZATION_LISTENER_H
