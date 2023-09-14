

#include "robot_localization/ros_robot_localization_listener.h"
#include "robot_localization/GetState.h"

#include <string>

namespace RobotLocalization
{

class RobotLocalizationListenerNode
{
 public:
  RobotLocalizationListenerNode( )
  {
    service_ = n_.advertiseService("get_state", &RobotLocalizationListenerNode::getStateCallback, this);
  }

  std::string getService( )
  {
    return service_.getService( );
  }

 private:
  RosRobotLocalizationListener rll_;
  ros::NodeHandle n_;
  ros::ServiceServer service_;

  bool getStateCallback(robot_localization::GetState::Request &req,
                        robot_localization::GetState::Response &res)
  {
    Eigen::VectorXd state(STATE_SIZE);
    Eigen::MatrixXd covariance(STATE_SIZE, STATE_SIZE);

    if (!rll_.getState(req.time_stamp, req.frame_id, state, covariance))
    {
      ROS_ERROR("Robot Localization Listener Node: Listener instance returned false at getState call.");
      return false;
    }

    for (size_t i = 0; i < STATE_SIZE; i++)
    {
      res.state[i] = (*(state.data( ) + i));
    }

    for (size_t i = 0; i < STATE_SIZE * STATE_SIZE; i++)
    {
      res.covariance[i] = (*(covariance.data( ) + i));
    }

    ROS_DEBUG("Robot Localization Listener Node: Listener responded with state and covariance at the requested time.");
    return true;
  }
};

} // namespace RobotLocalization

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_localization_listener_node");

  RobotLocalization::RobotLocalizationListenerNode rlln;
  ROS_INFO_STREAM("Robot Localization Listener Node: Ready to handle GetState requests at " << rlln.getService( ));

  ros::spin( );

  return 0;
}
