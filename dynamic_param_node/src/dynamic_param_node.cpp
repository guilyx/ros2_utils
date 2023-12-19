#include "dynamic_reparam_node/dynamic_param_node.hpp"

namespace ros2_utils
{
  DynamicParamNode::DynamicParamNode(const std::string& node_name) : rclcpp::Node(node_name)
  {
    registerOnSetCallback();
  }

  void DynamicParamNode::registerOnSetCallback()
  {
    // Register the callback for parameter changes
    _parameter_callback_handle = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult {
          return onSetParameter(parameters);
        });
  }

  rcl_interfaces::msg::SetParametersResult
      DynamicParamNode::onSetParameter(const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& parameter : parameters) {
      RCLCPP_INFO(this->get_logger(), "Parameter '%s' had its value changed to: %s", parameter.get_name().c_str(),
                  parameter.value_to_string().c_str());

      updateInternalAttribute(parameter);
    }

    return result;
  }
} // namespace ros2_utils