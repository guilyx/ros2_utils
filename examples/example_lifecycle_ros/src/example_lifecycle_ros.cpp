#include "example_lifecycle_ros/example_lifecycle_ros.h"
#include "example_msgs/msg/error.hpp"

namespace example
{
  ExampleLifecycleNode::ExampleLifecycleNode(bool intra_process_comms)
      : rclcpp_lifecycle::LifecycleNode("example_node",
                                        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    // Declare and Populate Parameters
    this->declare_parameter("initial_value", 10);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      ExampleLifecycleNode::on_configure(const rclcpp_lifecycle::State&)
  {
    RCLCPP_INFO(this->get_logger(), "on_configure()...");

    int initial_value;
    this->get_parameter("initial_value", initial_value);
    //

    using namespace std::placeholders;

    // Initialize Pure C++ Class with Parameters passed down from ROS
    _example_ptr = std::make_shared<Example>(initial_value);

    // Subscriber with lambda function callback
    _subscription_ptr = this->create_subscription<std_msgs::msg::Int32>(
        "subbed_data", 10, std::bind(&ExampleLifecycleNode::subCallback, this, _1));

    // Publisher
    _publisher_ptr = this->create_publisher<std_msgs::msg::Int32>("to_pub_data", 10);

    // Timer (instantly cancel for no autostart)
    _timer_ptr = this->create_wall_timer(500ms, std::bind(&ExampleLifecycleNode::timerCallback, this));
    _timer_ptr->cancel(); // You HAVE TO autostart a timer in ros2. It's okay to cancel it instantly.

    // Service Example
    _operations_service = this->create_service<example_msgs::srv::Calculate>(
        "calculate", std::bind(&ExampleLifecycleNode::calculateCallback, this, _1, _2, _3));

    // Service for Start/Stop Example
    _trigger_service = this->create_service<std_srvs::srv::Trigger>(
        "trigger_cumulate", std::bind(&ExampleLifecycleNode::triggerCallback, this, _1, _2, _3));

    RCLCPP_INFO(get_logger(), "Example Node has been configured.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      ExampleLifecycleNode::on_activate(const rclcpp_lifecycle::State&)
  {
    RCLCPP_INFO(this->get_logger(), "on_activate()...");
    _publisher_ptr->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      ExampleLifecycleNode::on_deactivate(const rclcpp_lifecycle::State&)
  {
    RCLCPP_INFO(this->get_logger(), "on_deactivate()...");
    _publisher_ptr->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      ExampleLifecycleNode::on_cleanup(const rclcpp_lifecycle::State&)
  {
    RCLCPP_INFO(this->get_logger(), "on_cleanup()...");
    _example_ptr.reset();
    _operations_service.reset();
    _trigger_service.reset();
    _subscription_ptr.reset();
    _publisher_ptr.reset();
    _timer_ptr.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      ExampleLifecycleNode::on_shutdown(const rclcpp_lifecycle::State&)
  {
    RCLCPP_INFO(this->get_logger(), "on_shutdown()...");
    _example_ptr.reset();
    _operations_service.reset();
    _trigger_service.reset();
    _subscription_ptr.reset();
    _publisher_ptr.reset();
    _timer_ptr.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void ExampleLifecycleNode::subCallback(const std_msgs::msg::Int32::SharedPtr message)
  {
    _example_ptr->setSubbedData(message->data);
    _example_ptr->process();
  }

  void ExampleLifecycleNode::timerCallback()
  {
    publish();
  }

  void ExampleLifecycleNode::triggerCallback(const std::shared_ptr<rmw_request_id_t>,
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

  void ExampleLifecycleNode::calculateCallback(const std::shared_ptr<rmw_request_id_t>,
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

  void ExampleLifecycleNode::publish()
  {
    int data = _example_ptr->getPubData();
    std_msgs::msg::Int32 message;
    message.data = data;
    _publisher_ptr->publish(message);
  }
}; // namespace example
