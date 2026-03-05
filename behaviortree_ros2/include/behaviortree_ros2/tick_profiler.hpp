#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace BT
{
class TickProfiler
{
public:
  TickProfiler(const std::string& logger_name, const std::string& node_name)
  : logger_(rclcpp::get_logger(logger_name)), node_name_(node_name), start_(std::chrono::steady_clock::now())
  {
  }

  TickProfiler(const rclcpp::Logger& logger, const std::string& node_name)
  : logger_(logger), node_name_(node_name), start_(std::chrono::steady_clock::now())
  {
  }

  ~TickProfiler()
  {
    const auto elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_)
            .count();
    RCLCPP_INFO(logger_, "[%s] tick() took %.3f ms", node_name_.c_str(), elapsed_ms);
  }

private:
  rclcpp::Logger logger_;
  std::string node_name_;
  std::chrono::steady_clock::time_point start_;
};
}  // namespace BT
