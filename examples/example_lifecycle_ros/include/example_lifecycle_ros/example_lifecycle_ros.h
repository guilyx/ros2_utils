#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <map>
#include <std_msgs/msg/int32.hpp>
#include <chrono>

#include "example/example.h"
#include <example_msgs/msg/basic_msg.hpp>
#include <example_msgs/srv/calculate.hpp>
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace example
{
  using CalculateSrvPtr = rclcpp::Service<example_msgs::srv::Calculate>::SharedPtr;
  using TriggerSrvPtr = rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr;

  class ExampleLifecycleNode : public rclcpp_lifecycle::LifecycleNode
  {
  private:
    std::shared_ptr<Example> _example_ptr;

    CalculateSrvPtr _operations_service;
    TriggerSrvPtr _trigger_service;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _subscription_ptr;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr _publisher_ptr;
    rclcpp::TimerBase::SharedPtr _timer_ptr;

    void subCallback(const std_msgs::msg::Int32::SharedPtr message);
    void timerCallback();
    void calculateCallback(const std::shared_ptr<rmw_request_id_t>,
                           const std::shared_ptr<example_msgs::srv::Calculate::Request> request,
                           std::shared_ptr<example_msgs::srv::Calculate::Response> response);
    void triggerCallback(const std::shared_ptr<rmw_request_id_t>,
                         const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  public:
    explicit ExampleLifecycleNode(bool intra_process_comms = false);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State&);

    void publish();
  };

} // namespace example