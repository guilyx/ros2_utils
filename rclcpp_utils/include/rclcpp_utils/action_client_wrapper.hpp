#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace rclcpp_utils
{

  template <class ActionT> class ActionClientWrapper : public rclcpp::Node
  {
  public:
    using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;

    ActionClientWrapper(std::string client_node_name, std::string action_name,
                        std::chrono::duration action_timeout = 100ms)
        : Node(client_node_name)
    {
      _action_client = rclcpp_action::create_client<ActionT>(get_node_base_interface(), get_node_graph_interface(),
                                                             get_node_logging_interface(),
                                                             get_node_waitables_interface(), action_name);

      _action_timeout = std::chrono::milliseconds(action_timeout_ms);
      _action_name = action_name;
      _client_node_name = client_node_name;
    }

    bool is_server_ready()
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Failed to check if action server is responding: ROS isn't running");
        return false;
      }
      return _action_client->action_server_is_ready();
    }

    bool is_goal_done() { return _is_goal_done; }
    bool is_action_running() { return _is_action_running; }

    void send_goal_async(const typename ActionT::Goal& goal)
    {
      if (!_action_client->wait_for_action_server(_action_timeout)) {
        RCLCPP_ERROR(get_logger(), "Action server did not respond in time: failed to send goal");
        return rclcpp_action::ResultCode::ABORTED;
      }
      auto send_goal_options = rclcpp_action::Client<ActionT>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&ActionClientWrapper::goal_response_callback, this, _1);
      send_goal_options.feedback_callback = std::bind(&ActionClientWrapper::feedback_callback, this, _1, _2);
      send_goal_options.result_callback = std::bind(&ActionClientWrapper::result_callback, this, _1);
      auto goal_handle_future = _action_client->async_send_goal(goal, send_goal_options);
    }

    bool get_action_result(ActionT::Result::SharedPtr result)
    {
      if (!is_goal_done() || !_action_result) {
        RCLCPP_ERROR(get_logger(), "Requested action result but action hasn't completed goal yet!");
        return false;
      }

      result = std::move(_action_result);
      return true;
    }

    bool get_action_feedback(std::shared_ptr<const ActionT::Feedback> feedback)
    {
      if (!is_action_running() || !_action_feedback) {
        RCLCPP_ERROR(get_logger(), "Requested action feedback but action isn't running yet!");
        return false;
      }

      feedback = std::move(_action_feedback);
      return true;
    }

    void cancel_goal()
    {
      auto future_cancel = _action_client->async_cancel_goal(_action_goal_handle);
      if (rclcpp::spin_until_future_complete(get_node_base_interface(), future_cancel) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to cancel action.");
        return;
      }

      RCLCPP_INFO(get_logger(), "Successfully requested to cancel goal.");
    }

  private:
    void goal_response_callback(GoalHandleT::SharedPtr goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        _is_action_running = true;
      }
    }

    void feedback_callback(GoalHandleT::SharedPtr, const std::shared_ptr<const ActionT::Feedback> feedback)
    {
      _action_feedback = feedback;
    }

    void result_callback(const GoalHandleT::WrappedResult& result)
    {
      this->goal_done_ = true;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED :
          break;
        case rclcpp_action::ResultCode::ABORTED :
          RCLCPP_ERROR(get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED :
          RCLCPP_ERROR(get_logger(), "Goal was canceled");
          return;
        default :
          RCLCPP_ERROR(get_logger(), "Unknown result code");
          return;
      }

      RCLCPP_INFO(get_logger(), "Result received");
      _action_result = result.result;
      _is_action_running = false;
    }

    typename rclcpp_action::Client<ActionT>::SharedPtr _action_client;

    bool _is_goal_done;
    bool _is_action_running;

    ActionT::Result::SharedPtr _action_result;
    std::shared_ptr<const ActionT::Feedback> _action_feedback;

    std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>> _action_goal_handle;
    std::string _action_name;
    std::chrono::milliseconds _action_timeout;
    std::string _client_node_name;
  };

} // namespace rclcpp_utils