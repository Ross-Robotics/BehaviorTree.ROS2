#include <gtest/gtest.h>

#include "behaviortree_ros2/ros_node_params.hpp"

TEST(RosNodeParams, DefaultsToLegacyCallbackExecution)
{
  const BT::RosNodeParams params;

  EXPECT_EQ(params.callback_execution_mode, BT::RosCallbackExecutionMode::Legacy);
}

TEST(RosNodeParams, SupportsOptingIntoReducedPolling)
{
  BT::RosNodeParams params;
  params.callback_execution_mode = BT::RosCallbackExecutionMode::ReducedPolling;

  EXPECT_EQ(params.callback_execution_mode, BT::RosCallbackExecutionMode::ReducedPolling);
}

TEST(RosNodeParams, SupportsSharedExecutorMode)
{
  BT::RosNodeParams params;
  params.callback_execution_mode = BT::RosCallbackExecutionMode::SharedExecutor;

  EXPECT_EQ(params.callback_execution_mode, BT::RosCallbackExecutionMode::SharedExecutor);
}
