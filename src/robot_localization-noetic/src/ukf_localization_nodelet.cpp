

#include "robot_localization/ros_filter_types.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <memory>
#include <vector>

namespace RobotLocalization
{

class UkfNodelet : public nodelet::Nodelet
{
 private:
  std::unique_ptr<RosUkf> ukf;

 public:
  virtual void onInit( )
  {
    NODELET_DEBUG("Initializing nodelet...");

    ros::NodeHandle nh      = getNodeHandle( );
    ros::NodeHandle nh_priv = getPrivateNodeHandle( );

    std::vector<double> args(3, 0);

    nh_priv.param("alpha", args[0], 0.001);
    nh_priv.param("kappa", args[1], 0.0);
    nh_priv.param("beta", args[2], 2.0);

    ukf = std::make_unique<RosUkf>(nh, nh_priv, getName( ), args);
    ukf->initialize( );
  }
};

} // namespace RobotLocalization

PLUGINLIB_EXPORT_CLASS(RobotLocalization::UkfNodelet, nodelet::Nodelet);
