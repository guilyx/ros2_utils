#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

namespace rclcpp_utils
{
  using namespace std::chrono_literals;

  template <typename ServiceT> class AsyncServiceClient : public rclcpp::Node
  {
  public:
    using ResponseCallback = std::function<void(typename ServiceT::Response::SharedPtr)>;

    AsyncServiceClient(const std::string& node_name, const std::string& service_name,
                       std::chrono::milliseconds service_timeout, ResponseCallback response_callback)
        : rclcpp::Node(node_name), service_name_(service_name), response_callback_(response_callback),
          service_timeout_(service_timeout)
    {
      client_ = this->create_client<ServiceT>(service_name_);
    }

    void send_request(typename ServiceT::Request::SharedPtr request)
    {
      while (!client_->wait_for_service(service_timeout_)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for service %s to appear...", service_name_.c_str());
      }

      auto result_future = client_->async_send_request(
          request, [this](rclcpp::Client<ServiceT>::SharedFuture future) { this->response_callback_(future.get()); });
    }

  private:
    std::string service_name_;
    typename rclcpp::Client<ServiceT>::SharedPtr client_;
    std::chrono::milliseconds service_timeout_;
    ResponseCallback response_callback_;
  };

} // namespace rclcpp_utils