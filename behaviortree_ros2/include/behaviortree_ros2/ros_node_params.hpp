// Copyright (c) 2023 Davide Faconti, Unmanned Life
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace BT
{

enum class RosCallbackExecutionMode
{
  Legacy,
  ReducedPolling,
  SharedExecutor
};

class RosCallbackExecutor
{
public:
  RosCallbackExecutor()
  : executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
    thread_ = std::thread([this]() { executor_->spin(); });
  }

  ~RosCallbackExecutor()
  {
    executor_->cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  RosCallbackExecutor(const RosCallbackExecutor&) = delete;
  RosCallbackExecutor& operator=(const RosCallbackExecutor&) = delete;

  void add_callback_group(
    const rclcpp::CallbackGroup::SharedPtr& callback_group,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr& node_base_interface)
  {
    executor_->add_callback_group(callback_group, node_base_interface);
  }

private:
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
};

struct RosNodeParams
{
  std::weak_ptr<rclcpp::Node> nh;

  // Preserve the existing private-executor polling behavior by default.
  RosCallbackExecutionMode callback_execution_mode = RosCallbackExecutionMode::Legacy;

  // Used only by SharedExecutor mode. The owner controls its lifetime.
  std::shared_ptr<RosCallbackExecutor> callback_executor;

  // This has different meaning based on the context:
  //
  // - RosActionNode: name of the action server
  // - RosServiceNode: name of the service
  // - RosTopicPubNode: name of the topic to publish to
  // - RosTopicSubNode: name of the topic to subscribe to
  std::string default_port_value;

  // parameters used only by service client and action clients

  // timeout when sending a request
  std::chrono::milliseconds server_timeout = std::chrono::milliseconds(1000);
  // timeout used when detecting the server the first time
  std::chrono::milliseconds wait_for_server_timeout = std::chrono::milliseconds(500);
};

}  // namespace BT
