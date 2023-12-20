#pragma once

#include "rclcpp/rclcpp.hpp"

namespace ros2_utils
{
  class DynamicParamNode : public rclcpp::Node
  {
  public:
    DynamicParamNode(const std::string& node_name);

  protected:
    void registerOnSetCallback();
    virtual void updateInternalAttribute(const rclcpp::Parameter& parameter) = 0;

  private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _parameter_callback_handle;

    // Callback for dynamic parameter changes
    rcl_interfaces::msg::SetParametersResult onSetParameter(const std::vector<rclcpp::Parameter>& parameters);
  };
} // namespace ros2_utils