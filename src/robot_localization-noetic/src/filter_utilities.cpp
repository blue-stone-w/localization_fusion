

#include "robot_localization/filter_utilities.h"
#include "robot_localization/filter_common.h"

#include <string>
#include <vector>

std::ostream &operator<<(std::ostream &os, const Eigen::MatrixXd &mat)
{
  os << "[";

  int rowCount = static_cast<int>(mat.rows( ));

  for (int row = 0; row < rowCount; ++row)
  {
    if (row > 0)
    {
      os << " ";
    }

    for (int col = 0; col < mat.cols( ); ++col)
    {
      os << std::setiosflags(std::ios::left) << std::setw(12) << std::setprecision(5) << mat(row, col);
    }

    if (row < rowCount - 1)
    {
      os << "\n";
    }
  }

  os << "]\n";

  return os;
}

std::ostream &operator<<(std::ostream &os, const Eigen::VectorXd &vec)
{
  os << "[";
  for (int dim = 0; dim < vec.rows( ); ++dim)
  {
    os << std::setiosflags(std::ios::left) << std::setw(12) << std::setprecision(5) << vec(dim);
  }
  os << "]\n";

  return os;
}

std::ostream &operator<<(std::ostream &os, const std::vector<size_t> &vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size( ); ++dim)
  {
    os << std::setiosflags(std::ios::left) << std::setw(12) << std::setprecision(5) << vec[dim];
  }
  os << "]\n";

  return os;
}

std::ostream &operator<<(std::ostream &os, const std::vector<int> &vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size( ); ++dim)
  {
    os << std::setiosflags(std::ios::left) << std::setw(3) << (vec[dim] ? "t" : "f");
  }
  os << "]\n";

  return os;
}

namespace RobotLocalization
{

namespace FilterUtilities
{
void appendPrefix(std::string tfPrefix, std::string &frameId)
{
  // Strip all leading slashes for tf2 compliance
  if (!frameId.empty( ) && frameId.at(0) == '/')
  {
    frameId = frameId.substr(1);
  }

  if (!tfPrefix.empty( ) && tfPrefix.at(0) == '/')
  {
    tfPrefix = tfPrefix.substr(1);
  }

  // If we do have a tf prefix, then put a slash in between
  if (!tfPrefix.empty( ))
  {
    frameId = tfPrefix + "/" + frameId;
  }
}

double clampRotation(double rotation)
{
  while (rotation > PI)
  {
    rotation -= TAU;
  }

  while (rotation < -PI)
  {
    rotation += TAU;
  }

  return rotation;
}

} // namespace FilterUtilities

} // namespace RobotLocalization
