#pragma once

#include <XmlRpcValue.h>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace mujoco_ros {
namespace detail {

// roslaunch expands slash-containing YAML keys into nested ROS parameters.
// Reconstruct the MuJoCo name while keeping the upstream string-value format.
inline bool ParseInitialJointMap(const XmlRpc::XmlRpcValue &value,
                                std::map<std::string, std::vector<double>> &result,
                                std::string &error, const std::string &prefix = "")
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    error = "joint_map must be a dictionary";
    return false;
  }
  std::map<std::string, std::vector<double>> parsed;
  for (auto entry = value.begin(); entry != value.end(); ++entry) {
    const std::string name = prefix + entry->first;
    if (entry->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      if (!ParseInitialJointMap(entry->second, parsed, error, name + "/")) {
        return false;
      }
      continue;
    }
    if (entry->second.getType() != XmlRpc::XmlRpcValue::TypeString) {
      error = "joint '" + name + "' must have a string value";
      return false;
    }
    std::istringstream stream(static_cast<const std::string &>(entry->second));
    std::vector<double> axes;
    double axis;
    while (stream >> axis) {
      if (!std::isfinite(axis)) {
        error = "joint '" + name + "' must contain finite numbers";
        return false;
      }
      axes.push_back(axis);
    }
    if (!stream.eof() || axes.empty()) {
      error = "joint '" + name + "' must contain space-separated numbers";
      return false;
    }
    parsed[name] = axes;
  }
  result.insert(parsed.begin(), parsed.end());
  return true;
}

} // namespace detail
} // namespace mujoco_ros
