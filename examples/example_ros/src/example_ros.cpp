#include "example_ros/example_ros.h"
#include "example_msgs/msg/error.hpp"

namespace example
{
  ExampleNode::ExampleNode() : Node("example_node")
  {
    using namespace std::placeholders;

    // Declare and Populate Parameters
    this->declare_parameter("initial_value", 10);
    int initial_value;
    this->get_parameter("initial_value", initial_value);
    //

    // Initialize Pure C++ Class with Parameters passed down from ROS
    _example_ptr = std::make_shared<Example>(initial_value);

    // Subscriber with lambda function callback
    _subscription_ptr = this->create_subscription<std_msgs::msg::Int32>(
        "subbed_data", 10, [this](const std_msgs::msg::Int32::SharedPtr message) { subCallback(message); });

    // Publisher
    _publisher_ptr = this->create_publisher<std_msgs::msg::Int32>("to_pub_data", 10);

    // Timer (instantly cancel for no autostart)
    _timer_ptr = this->create_wall_timer(500ms, std::bind(&ExampleNode::timerCallback, this));
    _timer_ptr->cancel(); // You HAVE TO autostart a timer in ros2. It's okay to cancel it instantly.

    // Service Example
    _operations_service = this->create_service<example_msgs::srv::Calculate>(
        "calculate", std::bind(&ExampleNode::calculateCallback, this, _1, _2, _3));

    // Service for Start/Stop Example
    _trigger_service = this->create_service<std_srvs::srv::Trigger>(
        "trigger_cumulate", std::bind(&ExampleNode::triggerCallback, this, _1, _2, _3));

    RCLCPP_INFO(get_logger(), "Example Node has been initialized.");
  }

  void ExampleNode::subCallback(const std_msgs::msg::Int32::SharedPtr message)
  {
    _example_ptr->setSubbedData(message->data);
    _example_ptr->process();
  }

  void ExampleNode::timerCallback()
  {
    publish();
  }

  void ExampleNode::triggerCallback(const std::shared_ptr<rmw_request_id_t>,
                                    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      if (_timer_ptr->is_canceled()) // if stopped
      {
        _timer_ptr->reset(); // start
        response->message = "Start Publishing.";
        RCLCPP_INFO(get_logger(), "Example Node starts publishing.");
      } else {
        _timer_ptr->cancel(); // stop
        response->message = "Stop Publishing.";
        RCLCPP_INFO(get_logger(), "Example Node stops publishing.");
      }
      response->success = true;
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Example Node trigger has failed: %s", ex.what());
      response->success = false;
      response->message = std::string(ex.what());
    }
  }

  void ExampleNode::calculateCallback(const std::shared_ptr<rmw_request_id_t>,
                                      const std::shared_ptr<example_msgs::srv::Calculate::Request> request,
                                      std::shared_ptr<example_msgs::srv::Calculate::Response> response)
  {
    try {
      auto op = static_cast<OperationT>(request->op_type);
      RCLCPP_INFO(get_logger(), "Received request for [%s]", toString(op).c_str());
      response->result = _example_ptr->calculate(op, request->input.first, request->input.second);
    } catch (const std::exception& ex) {
      example_msgs::msg::Error err;
      err.code = 405;
      err.message = std::string(ex.what());
      response->error.push_back(err);
    }
  }

  void ExampleNode::publish()
  {
    int data = _example_ptr->getPubData();
    std_msgs::msg::Int32 message;
    message.data = data;
    _publisher_ptr->publish(message);
  }
}; // namespace example
