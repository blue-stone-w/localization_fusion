

#include "robot_localization/ros_filter_types.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <memory>

namespace RobotLocalization
{

class EkfNodelet : public nodelet::Nodelet
{
 private:
  std::unique_ptr<RosEkf> ekf;

 public:
  virtual void onInit( )
  {
    NODELET_DEBUG("Initializing nodelet...");

    ros::NodeHandle nh      = getNodeHandle( );
    ros::NodeHandle nh_priv = getPrivateNodeHandle( );

    ekf = std::make_unique<RosEkf>(nh, nh_priv, getName( ));
    ekf->initialize( );
  }
};

} // namespace RobotLocalization

PLUGINLIB_EXPORT_CLASS(RobotLocalization::EkfNodelet, nodelet::Nodelet);
