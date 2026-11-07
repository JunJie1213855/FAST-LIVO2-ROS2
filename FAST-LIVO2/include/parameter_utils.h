#ifndef FAST_LIVO_PARAMETER_UTILS_H_
#define FAST_LIVO_PARAMETER_UTILS_H_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace fast_livo
{
template <typename T>
T GetOrDeclareParameter(const rclcpp::Node::SharedPtr &node, const std::string &name, const T &default_value)
{
  if (!node->has_parameter(name))
    return node->declare_parameter<T>(name, default_value);
  return node->get_parameter(name).get_value<T>();
}
}  // namespace fast_livo

#endif  // FAST_LIVO_PARAMETER_UTILS_H_
