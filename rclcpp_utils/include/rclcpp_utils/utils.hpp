#pragma once

#include <rclcpp/rclcpp.hpp>

namespace rclcpp_utils
{
  template <class T> void declare_get_parameter(rclcpp::Node& node, const std::string& name, T& val, const T& def)
  {
    if (!node.has_parameter(name))
      node.declare_parameter(name, def);
    node.get_parameter(name, val);
  }
} // namespace rclcpp_utils