

#ifndef ROBOT_LOCALIZATION_FILTER_UTILITIES_H
#define ROBOT_LOCALIZATION_FILTER_UTILITIES_H

#include <Eigen/Dense>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#define FB_DEBUG(msg)     \
  if (getDebug( ))        \
  {                       \
    *debugStream_ << msg; \
  }

// Handy methods for debug output
std::ostream &operator<<(std::ostream &os, const Eigen::MatrixXd &mat);
std::ostream &operator<<(std::ostream &os, const Eigen::VectorXd &vec);
std::ostream &operator<<(std::ostream &os, const std::vector<size_t> &vec);
std::ostream &operator<<(std::ostream &os, const std::vector<int> &vec);

namespace RobotLocalization
{
namespace FilterUtilities
{

//! @brief Utility method keeping RPY angles in the range [-pi, pi]
//! @param[in] rotation - The rotation to bind
//! @return the bounded value
double clampRotation(double rotation);

//! @brief Utility method for appending tf2 prefixes cleanly
//! @param[in] tfPrefix - the tf2 prefix to append
//! @param[in, out] frameId - the resulting frame_id value
void appendPrefix(std::string tfPrefix, std::string &frameId);

} // namespace FilterUtilities
} // namespace RobotLocalization

#endif // ROBOT_LOCALIZATION_FILTER_UTILITIES_H
