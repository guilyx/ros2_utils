#pragma once

#include "rclcpp/rclcpp.hpp"

namespace ros2_utils
{
  class DynamicParamNode : public rclcpp::Node
  {
  public:
    inline DynamicParamNode(const std::string& node_name) : rclcpp::Node(node_name) { registerOnSetCallback(); }

  protected:
    inline void registerOnSetCallback()
    {
      // Register the callback for parameter changes
      _parameter_callback_handle = this->add_on_set_parameters_callback(
          [this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult {
            return onSetParameter(parameters);
          });
    }

    virtual void updateInternalAttribute(const rclcpp::Parameter& parameter) = 0;

  private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _parameter_callback_handle;

    // Callback for dynamic parameter changes
    rcl_interfaces::msg::SetParametersResult onSetParameter(const std::vector<rclcpp::Parameter>& parameters);
  };
} // namespace ros2_utils