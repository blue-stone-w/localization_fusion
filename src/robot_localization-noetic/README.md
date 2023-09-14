robot_localization
==================

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://docs.ros.org/noetic/api/robot_localization/html/index.html

branch noetic-devel


|filter_utilities.h|limit angle and combine tf and frameid|
|filter_utilities.cpp||
|filter_common.h|define state vector, control vector, state's demension|
|filter_base.h|define measure, filter state, |
|||
|ekf.h||
|ukf.h||
|||
|robot_localization_estimator.h||
|ros_robot_localization_listener.h||
|||
|ros_filter_utilities.h||
|ros_filter.h||
|ros_filter_types.h||
|||
|navsat_conversions.h||
|navsat_transform.h||
|navsat_transform.cpp||
|navsat_transform_node.cpp||
|navsat_transform_nodelet.cpp||