#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/execute_tree.hpp"

namespace
{
using Action = btcpp_ros2_interfaces::action::ExecuteTree;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

class TestActionNode : public BT::RosActionNode<Action>
{
public:
  TestActionNode(const std::string& name, const BT::NodeConfig& config,
                 const BT::RosNodeParams& params)
    : RosActionNode(name, config, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool setGoal(Goal& goal) override
  {
    goal.target_tree = "test_tree";
    goal.payload = "test_payload";
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult&) override
  {
    result_received = true;
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback>) override
  {
    ++feedback_count;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    failure = error;
    return BT::NodeStatus::FAILURE;
  }

  bool result_received{ false };
  int feedback_count{ 0 };
  BT::ActionNodeErrorCode failure{ BT::INVALID_GOAL };
};

BT::NodeStatus run_action(BT::RosCallbackExecutionMode mode, int& feedback_count)
{
  const auto suffix = mode == BT::RosCallbackExecutionMode::Legacy ? "legacy" : "shared";
  const auto action_name = std::string("/bt_action_callback_test_") + suffix;
  auto server_node =
      std::make_shared<rclcpp::Node>("bt_action_server_" + std::string(suffix));
  auto client_node =
      std::make_shared<rclcpp::Node>("bt_action_client_" + std::string(suffix));
  std::atomic_bool server_completed{ false };

  auto server = rclcpp_action::create_server<Action>(
      server_node, action_name,
      [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&server_completed](const std::shared_ptr<GoalHandle>& goal_handle) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        auto feedback = std::make_shared<Action::Feedback>();
        feedback->message = "working";
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto result = std::make_shared<Action::Result>();
        result->return_message = "complete";
        result->node_status.status = 2;
        goal_handle->succeed(result);
        server_completed = true;
      });

  rclcpp::executors::SingleThreadedExecutor server_executor;
  server_executor.add_node(server_node);
  std::thread server_thread([&server_executor]() { server_executor.spin(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  BT::NodeConfig config;
  config.input_ports["action_name"] = action_name;
  BT::RosNodeParams params;
  params.nh = client_node;
  params.callback_execution_mode = mode;
  if(mode == BT::RosCallbackExecutionMode::SharedExecutor)
  {
    params.callback_executor = std::make_shared<BT::RosCallbackExecutor>();
  }

  TestActionNode node("test_action", config, params);
  auto status = BT::NodeStatus::RUNNING;
  for(int attempt = 0; attempt < 200 && status == BT::NodeStatus::RUNNING; ++attempt)
  {
    status = node.executeTick();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  server_executor.cancel();
  server_thread.join();
  EXPECT_TRUE(server_completed.load());
  feedback_count = node.feedback_count;
  EXPECT_TRUE(node.result_received);
  return status;
}
}  // namespace

class RosActionCallbackModesTest : public ::testing::Test
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

TEST_F(RosActionCallbackModesTest, LegacyModeDeliversFeedbackAndResult)
{
  int feedback_count = 0;
  EXPECT_EQ(run_action(BT::RosCallbackExecutionMode::Legacy, feedback_count),
            BT::NodeStatus::SUCCESS);
  EXPECT_GT(feedback_count, 0);
}

TEST_F(RosActionCallbackModesTest, SharedExecutorModeDeliversFeedbackAndResult)
{
  int feedback_count = 0;
  EXPECT_EQ(run_action(BT::RosCallbackExecutionMode::SharedExecutor, feedback_count),
            BT::NodeStatus::SUCCESS);
  EXPECT_GT(feedback_count, 0);
}
