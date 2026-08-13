#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{
class TestSubscriberNode : public BT::RosTopicSubNode<std_msgs::msg::Bool>
{
public:
  TestSubscriberNode(const std::string& name, const BT::NodeConfig& config,
                     const BT::RosNodeParams& params)
    : RosTopicSubNode(name, config, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& message) override
  {
    received = message && message->data;
    return received ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  bool received{ false };
};

BT::NodeStatus run_subscriber(BT::RosCallbackExecutionMode mode)
{
  const auto suffix = mode == BT::RosCallbackExecutionMode::Legacy ? "legacy" : "shared";
  const auto topic_name = std::string("/bt_topic_callback_test_") + suffix;
  auto publisher_node =
      std::make_shared<rclcpp::Node>("bt_topic_publisher_" + std::string(suffix));
  auto subscriber_node =
      std::make_shared<rclcpp::Node>("bt_topic_subscriber_" + std::string(suffix));
  auto publisher = publisher_node->create_publisher<std_msgs::msg::Bool>(topic_name, 1);

  BT::NodeConfig config;
  config.input_ports["topic_name"] = topic_name;
  BT::RosNodeParams params;
  params.nh = subscriber_node;
  params.callback_execution_mode = mode;
  if(mode == BT::RosCallbackExecutionMode::SharedExecutor)
  {
    params.callback_executor = std::make_shared<BT::RosCallbackExecutor>();
  }

  TestSubscriberNode node("test_subscriber", config, params);
  std_msgs::msg::Bool message;
  message.data = true;
  auto status = BT::NodeStatus::FAILURE;
  for(int attempt = 0; attempt < 100 && status != BT::NodeStatus::SUCCESS; ++attempt)
  {
    publisher->publish(message);
    status = node.executeTick();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(node.received);
  return status;
}
}  // namespace

class RosTopicCallbackModesTest : public ::testing::Test
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

TEST_F(RosTopicCallbackModesTest, LegacyModeDeliversTopicMessage)
{
  EXPECT_EQ(run_subscriber(BT::RosCallbackExecutionMode::Legacy),
            BT::NodeStatus::SUCCESS);
}

TEST_F(RosTopicCallbackModesTest, SharedExecutorModeDeliversTopicMessage)
{
  EXPECT_EQ(run_subscriber(BT::RosCallbackExecutionMode::SharedExecutor),
            BT::NodeStatus::SUCCESS);
}
