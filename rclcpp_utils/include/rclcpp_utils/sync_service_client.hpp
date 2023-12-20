#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

namespace rclcpp_utils
{

  using namespace std::chrono_literals;

  template <class ServiceT> class SyncServiceClient
  {
  public:
    using SharedPtr = std::shared_ptr<SyncServiceClient<ServiceT>>;

    SyncServiceClient(rclcpp::Node* node_handle, std::string service_name,
                             std::chrono::milliseconds service_timeout = 100ms)
        : _nh(node_handle)
    {
      _service_client = _nh->create_client<ServiceT>(service_name);
      _service_timeout = service_timeout;
      _service_name = service_name;
    }

    bool is_server_ready()
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(_nh->get_logger(), "Failed to check if service server is responding: ROS isn't running");
        return false;
      }
      return _service_client->wait_for_service(_service_timeout);
    }

    std::shared_ptr<typename ServiceT::Response> send_request(const std::shared_ptr<typename ServiceT::Request> request)
    {
      if (!is_server_ready())
        return nullptr;

      auto res = _service_client->async_send_request(request);
      auto status = res.wait_for(_service_timeout);
      if (status != std::future_status::ready) {
        RCLCPP_ERROR(_nh->get_logger(), "Failed to call service.");
        return nullptr;
      }

      return res.get();
    }

  private:
    typename rclcpp::Client<ServiceT>::SharedPtr _service_client;
    std::string _service_name;
    std::chrono::milliseconds _service_timeout;
    rclcpp::Node* _nh;
  };
} // namespace rclcpp_utils
