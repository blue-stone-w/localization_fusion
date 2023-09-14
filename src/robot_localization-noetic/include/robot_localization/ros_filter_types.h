
#ifndef ROBOT_LOCALIZATION_ROS_FILTER_TYPES_H
#define ROBOT_LOCALIZATION_ROS_FILTER_TYPES_H

#include "robot_localization/ros_filter.h"
#include "robot_localization/ekf.h"
#include "robot_localization/ukf.h"

namespace RobotLocalization
{

typedef RosFilter<Ekf> RosEkf;
typedef RosFilter<Ukf> RosUkf;

} // namespace RobotLocalization

#endif // ROBOT_LOCALIZATION_ROS_FILTER_TYPES_H
