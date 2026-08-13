#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
class TestServiceNode : public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  TestServiceNode(const std::string& name, const BT::NodeConfig& config,
                  const BT::RosNodeParams& params)
    : RosServiceNode(name, config, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool setRequest(Request::SharedPtr& /*request*/) override
  {
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    response_received = response && response->success;
    return response_received ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    failure = error;
    return BT::NodeStatus::FAILURE;
  }

  bool response_received{ false };
  BT::ServiceNodeErrorCode failure{ BT::SERVICE_ABORTED };
};

BT::NodeStatus run_service_request(BT::RosCallbackExecutionMode mode)
{
  const auto suffix = mode == BT::RosCallbackExecutionMode::Legacy ? "legacy" : "shared";
  const auto service_name = std::string("/bt_service_callback_test_") + suffix;
  auto server_node =
      std::make_shared<rclcpp::Node>("bt_service_server_" + std::string(suffix));
  auto client_node =
      std::make_shared<rclcpp::Node>("bt_service_client_" + std::string(suffix));
  std::atomic_bool server_called{ false };
  auto service = server_node->create_service<std_srvs::srv::Trigger>(
      service_name,
      [&server_called](const std::shared_ptr<const std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        server_called = true;
        response->success = true;
        response->message = "ok";
      });
  EXPECT_NE(service, nullptr);
  if(!service)
  {
    return BT::NodeStatus::FAILURE;
  }

  rclcpp::executors::SingleThreadedExecutor server_executor;
  server_executor.add_node(server_node);
  std::thread server_thread([&server_executor]() { server_executor.spin(); });

  BT::NodeConfig config;
  config.input_ports["service_name"] = service_name;
  BT::RosNodeParams params;
  params.nh = client_node;
  params.callback_execution_mode = mode;
  if(mode == BT::RosCallbackExecutionMode::SharedExecutor)
  {
    params.callback_executor = std::make_shared<BT::RosCallbackExecutor>();
  }

  TestServiceNode node("test_service", config, params);
  auto status = BT::NodeStatus::RUNNING;
  for(int attempt = 0; attempt < 100 && status == BT::NodeStatus::RUNNING; ++attempt)
  {
    status = node.executeTick();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  server_executor.cancel();
  server_thread.join();
  EXPECT_TRUE(server_called.load());
  EXPECT_TRUE(node.response_received);
  return status;
}
}  // namespace

class RosServiceCallbackModesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(RosServiceCallbackModesTest, LegacyModeDeliversServiceResponse)
{
  EXPECT_EQ(run_service_request(BT::RosCallbackExecutionMode::Legacy),
            BT::NodeStatus::SUCCESS);
}

TEST_F(RosServiceCallbackModesTest, SharedExecutorModeDeliversServiceResponse)
{
  EXPECT_EQ(run_service_request(BT::RosCallbackExecutionMode::SharedExecutor),
            BT::NodeStatus::SUCCESS);
}
