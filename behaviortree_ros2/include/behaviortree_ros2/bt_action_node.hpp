// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2023 Davide Faconti
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

#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_ros2/ros_node_params.hpp"

namespace BT
{

enum ActionNodeErrorCode
{
  SERVER_UNREACHABLE,
  SEND_GOAL_TIMEOUT,
  GOAL_REJECTED_BY_SERVER,
  ACTION_ABORTED,
  ACTION_CANCELLED,
  INVALID_GOAL
};

inline const char* toStr(const ActionNodeErrorCode& err)
{
  switch(err)
  {
    case SERVER_UNREACHABLE:
      return "SERVER_UNREACHABLE";
    case SEND_GOAL_TIMEOUT:
      return "SEND_GOAL_TIMEOUT";
    case GOAL_REJECTED_BY_SERVER:
      return "GOAL_REJECTED_BY_SERVER";
    case ACTION_ABORTED:
      return "ACTION_ABORTED";
    case ACTION_CANCELLED:
      return "ACTION_CANCELLED";
    case INVALID_GOAL:
      return "INVALID_GOAL";
  }
  return nullptr;
}

/**
 * @brief Abstract class to wrap rclcpp_action::Client<>
 *
 * For instance, given the type AddTwoInts described in this tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
 *
 * the corresponding wrapper would be:
 *
 * class FibonacciNode: public RosActionNode<action_tutorials_interfaces::action::Fibonacci>
 *
 * RosActionNode will try to be non-blocking for the entire duration of the call.
 * The derived class must reimplement the virtual methods as described below.
 *
 * The name of the action will be determined as follows:
 *
 * 1. If a value is passes in the InputPort "action_name", use that
 * 2. Otherwise, use the value in RosNodeParams::default_port_value
 */
template <class ActionT>
class RosActionNode : public BT::ActionNodeBase
{
public:
  // Type definitions
  using ActionType = ActionT;
  using ActionClient = typename rclcpp_action::Client<ActionT>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;
  using Goal = typename ActionT::Goal;
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using Feedback = typename ActionT::Feedback;

  /** To register this class into the factory, use:
   *
   *    factory.registerNodeType<>(node_name, params);
   *
   */
  explicit RosActionNode(const std::string& instance_name, const BT::NodeConfig& conf,
                         const RosNodeParams& params);

  virtual ~RosActionNode()
  {
    if (client_instance_) {
      client_instance_.reset();
      std::unique_lock lk(getMutex());
      auto& registry = getRegistry();
      auto it = registry.find(action_client_key_);
      if (it != registry.end() && it->second.use_count() <= 1) {
        registry.erase(it);
        RCLCPP_INFO(logger(), "Removed action client [%s]", action_name_.c_str());
      }
    }
  }

  /**
   * @brief Any subclass of RosActionNode that has ports must implement a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return PortsList containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = { InputPort<std::string>("action_name", "__default__placeholder__",
                                               "Action server name") };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /// @brief  Callback executed when the node is halted. Note that cancelGoal()
  /// is done automatically.
  virtual void onHalt()
  {}

  /** setGoal s a callback that allows the user to set
   *  the goal message (ActionT::Goal).
   *
   * @param goal  the goal to be sent to the action server.
   *
   * @return false if the request should not be sent. In that case,
   * RosActionNode::onFailure(INVALID_GOAL) will be called.
   */
  virtual bool setGoal(Goal& goal) = 0;

  /** Callback invoked when the result is received by the server.
   * It is up to the user to define if the action returns SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onResultReceived(const WrappedResult& result) = 0;

  /** Callback invoked when the feedback is received.
   * It generally returns RUNNING, but the user can also use this callback to cancel the
   * current action and return SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*feedback*/)
  {
    return NodeStatus::RUNNING;
  }

  /** Callback invoked when something goes wrong.
   * It must return either SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFailure(ActionNodeErrorCode /*error*/)
  {
    return NodeStatus::FAILURE;
  }

  /// Method used to send a request to the Action server to cancel the current goal
  void cancelGoal();

  /// The default halt() implementation will call cancelGoal if necessary.
  void halt() override final;

  NodeStatus tick() override final;

protected:
  struct ActionClientInstance
  {
    ActionClientInstance(std::shared_ptr<rclcpp::Node> node,
                         const std::string& action_name,
                         const std::shared_ptr<RosCallbackExecutor>& shared_executor);

    ActionClientPtr action_client;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_executor;
    std::shared_ptr<RosCallbackExecutor> shared_executor;
    typename ActionClient::SendGoalOptions goal_options;
  };

  static std::mutex& getMutex()
  {
    static std::mutex action_client_mutex;
    return action_client_mutex;
  }

  rclcpp::Logger logger()
  {
    if(auto node = node_.lock())
    {
      return node->get_logger();
    }
    return rclcpp::get_logger("RosActionNode");
  }

  rclcpp::Time now()
  {
    if(auto node = node_.lock())
    {
      return node->now();
    }
    return rclcpp::Clock(RCL_ROS_TIME).now();
  }

  using ClientsRegistry =
      std::unordered_map<std::string, std::shared_ptr<ActionClientInstance>>;
  // contains the fully-qualified name of the node and the name of the client
  static ClientsRegistry& getRegistry()
  {
    static ClientsRegistry action_clients_registry;
    return action_clients_registry;
  }

  std::weak_ptr<rclcpp::Node> node_;
  std::shared_ptr<ActionClientInstance> client_instance_;
  std::string action_name_;
  bool action_name_may_change_ = false;
  RosCallbackExecutionMode callback_execution_mode_;
  std::shared_ptr<RosCallbackExecutor> callback_executor_;
  std::chrono::milliseconds server_timeout_;
  const std::chrono::milliseconds wait_for_server_timeout_;
  std::string action_client_key_;

private:
  std::shared_future<typename GoalHandle::SharedPtr> future_goal_handle_;
  typename GoalHandle::SharedPtr goal_handle_;
  rclcpp::Time time_goal_sent_;
  NodeStatus on_feedback_state_change_;
  std::mutex on_feedback_state_change_mutex_;
  bool goal_received_;
  WrappedResult result_;
  WrappedResult pending_result_;   // holds a result that arrived before goal_handle_ was set
  bool has_pending_result_ = false;
  bool cancellation_requested_ = false;
  std::deque<std::shared_ptr<const Feedback>> pending_feedback_;
  // Single mutex protecting goal_handle_, goal_received_, result_, pending_result_, and
  // has_pending_result_ together. They form one state machine, so one lock avoids
  // ordering issues and ensures consistent snapshots when reading in tick().
  std::mutex action_state_mutex_;

  bool createClient(const std::string& action_name);

  bool checkActionClient();
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class T>
RosActionNode<T>::ActionClientInstance::ActionClientInstance(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& action_name,
    const std::shared_ptr<RosCallbackExecutor>& shared_executor)
{
  callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->shared_executor = shared_executor;
  if(this->shared_executor) {
    this->shared_executor->add_callback_group(callback_group, node->get_node_base_interface());
  } else {
    callback_executor.add_callback_group(callback_group, node->get_node_base_interface());
  }
  action_client = rclcpp_action::create_client<T>(node, action_name, callback_group);
}

template <class T>
inline RosActionNode<T>::RosActionNode(const std::string& instance_name,
                                       const NodeConfig& conf,
                                       const RosNodeParams& params)
  : BT::ActionNodeBase(instance_name, conf)
  , node_(params.nh)
  , callback_execution_mode_(params.callback_execution_mode)
  , callback_executor_(params.callback_executor)
  , server_timeout_(params.server_timeout)
  , wait_for_server_timeout_(params.wait_for_server_timeout)
{
  // Three cases:
  // - we use the default action_name in RosNodeParams when port is empty
  // - we use the action_name in the port and it is a static string.
  // - we use the action_name in the port and it is blackboard entry.

  // check port remapping
  auto portIt = config().input_ports.find("action_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_action_name = portIt->second;

    if(bb_action_name.empty() || bb_action_name == "__default__placeholder__")
    {
      if(params.default_port_value.empty())
      {
        throw std::logic_error("Both [action_name] in the InputPort and the "
                               "RosNodeParams are empty.");
      }
      else
      {
        createClient(params.default_port_value);
      }
    }
    else if(!isBlackboardPointer(bb_action_name))
    {
      // If the content of the port "action_name" is not
      // a pointer to the blackboard, but a static string, we can
      // create the client in the constructor.
      createClient(bb_action_name);
    }
    else
    {
      action_name_may_change_ = true;
      // createClient will be invoked in the first tick().
    }
  }
  else
  {
    if(params.default_port_value.empty())
    {
      throw std::logic_error("Both [action_name] in the InputPort and the RosNodeParams "
                             "are empty.");
    }
    else
    {
      createClient(params.default_port_value);
    }
  }
}

template <class T>
inline bool RosActionNode<T>::createClient(const std::string& action_name)
{
  if(action_name.empty())
  {
    throw RuntimeError("action_name is empty");
  }

  std::unique_lock lk(getMutex());
  auto node = node_.lock();
  if(!node)
  {
    throw RuntimeError("The ROS node went out of scope. RosNodeParams doesn't take the "
                       "ownership of the node.");
  }
  if(callback_execution_mode_ == RosCallbackExecutionMode::SharedExecutor &&
     !callback_executor_)
  {
    throw RuntimeError(
      "SharedExecutor callback mode requires RosNodeParams::callback_executor.");
  }
  action_client_key_ = std::string(node->get_fully_qualified_name()) + "/" + action_name;
  if(callback_execution_mode_ == RosCallbackExecutionMode::SharedExecutor) {
    action_client_key_ += "/shared_executor";
  }

  auto& registry = getRegistry();
  auto it = registry.find(action_client_key_);
  if(it == registry.end())
  {
    const auto shared_executor =
      callback_execution_mode_ == RosCallbackExecutionMode::SharedExecutor ?
      callback_executor_ : std::shared_ptr<RosCallbackExecutor>();
    client_instance_ = std::make_shared<ActionClientInstance>(
      node, action_name, shared_executor);
    registry.insert({ action_client_key_, client_instance_ });

    RCLCPP_INFO(logger(), "Node [%s] created action client [%s]", name().c_str(),
                action_name.c_str());
  }
  else
  {
    client_instance_ = it->second;
  }

  action_name_ = action_name;

  return true;
}

template <class T>
inline NodeStatus RosActionNode<T>::tick()
{
  if(!rclcpp::ok())
  {
    halt();
    return NodeStatus::FAILURE;
  }

  // First, check if the action_client_ is valid and that the name of the
  // action_name in the port didn't change.
  // otherwise, create a new client
  if(!client_instance_ || (status() == NodeStatus::IDLE && action_name_may_change_))
  {
    std::string action_name;
    getInput("action_name", action_name);
    if(action_name_ != action_name)
    {
      createClient(action_name);
    }
  }
  auto& action_client = client_instance_->action_client;

  //------------------------------------------
  auto CheckStatus = [](NodeStatus status) {
    if(!isStatusCompleted(status))
    {
      throw LogicError("RosActionNode: the callback must return either SUCCESS nor "
                       "FAILURE");
    }
    return status;
  };

  // first step to be done only at the beginning of the Action
  if(status() == BT::NodeStatus::IDLE)
  {
    setStatus(NodeStatus::RUNNING);

    {
      std::lock_guard<std::mutex> state_lock(action_state_mutex_);
      goal_handle_.reset();
      goal_received_ = false;
      result_ = {};
      pending_result_ = {};
      has_pending_result_ = false;
      cancellation_requested_ = false;
    }
    future_goal_handle_ = {};
    {
      std::lock_guard<std::mutex> feedback_lock(on_feedback_state_change_mutex_);
      on_feedback_state_change_ = NodeStatus::RUNNING;
      pending_feedback_.clear();
    }

    Goal goal;

    if(!setGoal(goal))
    {
      return CheckStatus(onFailure(INVALID_GOAL));
    }

    typename ActionClient::SendGoalOptions goal_options;

    //--------------------
    goal_options.feedback_callback =
        [this](typename GoalHandle::SharedPtr,
               const std::shared_ptr<const Feedback> feedback) {
          std::lock_guard<std::mutex> lock(on_feedback_state_change_mutex_);
          if(callback_execution_mode_ == RosCallbackExecutionMode::SharedExecutor)
          {
            pending_feedback_.push_back(feedback);
          }
          else
          {
            on_feedback_state_change_ = onFeedback(feedback);
            if(on_feedback_state_change_ == NodeStatus::IDLE)
            {
              throw std::logic_error("onFeedback must not return IDLE");
            }
          }
          emitWakeUpSignal();
        };
    //--------------------
    // goal_response_callback normally provides the accepted goal handle before the
    // terminal result is processed. If a result is observed before goal_handle_ is
    // available, result_callback stores it as pending and this callback validates it
    // against the accepted goal_id.
    goal_options.goal_response_callback =
        [this](typename GoalHandle::SharedPtr handle) {
          bool should_cancel = false;
          if (handle) {
            {
              std::lock_guard<std::mutex> state_lock(action_state_mutex_);
              goal_handle_ = handle;
              goal_received_ = true;
              should_cancel = cancellation_requested_;

              // If a result arrived before this callback (fast server abort race), validate
              // it now that we have the goal handle and can check the goal ID.
              if (has_pending_result_) {
                if (pending_result_.goal_id == handle->get_goal_id()) {
                  result_ = pending_result_;
                  goal_handle_.reset();
                } else {
                  RCLCPP_WARN(
                    logger(),
                    "Discarding pending early result for [%s]: goal_id does not match "
                    "accepted goal.",
                    action_name_.c_str());
                }
                pending_result_ = {};
                has_pending_result_ = false;
              }
            }
            if(should_cancel) {
              try {
                client_instance_->action_client->async_cancel_goal(handle);
              } catch(const std::exception& e) {
                RCLCPP_WARN(
                  logger(), "Failed to cancel late-accepted goal [%s]: %s",
                  action_name_.c_str(), e.what());
              }
            }
          }
          // If null the goal was rejected; GOAL_REJECTED_BY_SERVER is handled in tick().
          emitWakeUpSignal();
        };
    //--------------------
    goal_options.result_callback = [this](const WrappedResult& result) {
    try {
      bool should_wake = false;

      {
        std::lock_guard<std::mutex> state_lock(action_state_mutex_);

        if (!goal_handle_) {
          // goal_handle_ is null. Two possible causes:
          // (a) The result arrived before goal_response_callback populated goal_handle_
          //     (fast server abort). Store it; goal_response_callback or tick()'s FIRST
          //     case will validate it against the accepted goal_id before promoting it.
          // (b) Stale result from a previous goal whose handle was already cleared.
          //     The goal_id check in goal_response_callback will discard it if stale.
          // Either way, never ignore — (a) would cause the node to get stuck.
          if (!has_pending_result_) {
            RCLCPP_WARN(
              logger(),
              "Received result for [%s] while goal_handle_ is empty. Storing as pending "
              "for goal_id validation.",
              action_name_.c_str());
            pending_result_ = result;
            has_pending_result_ = true;
            should_wake = true;
          } else {
            RCLCPP_WARN(
              logger(),
              "Received result for [%s] but a pending result already exists. Ignoring.",
              action_name_.c_str());
          }
        } else if (goal_handle_->get_goal_id() != result.goal_id) {
          RCLCPP_WARN(
            logger(),
            "Received result for [%s] but goal_id does not match active goal. Ignoring.",
            action_name_.c_str());
        } else {
          // Store result and clear the handle atomically under the same lock so that
          // no other reader can see a partial state (result set but handle not yet
          // cleared or vice versa).
          result_ = result;
          goal_handle_.reset();
          should_wake = true;
        }
      }

      if (should_wake) {
        RCLCPP_DEBUG(logger(), "Stored terminal/pending result for [%s]", action_name_.c_str());
        emitWakeUpSignal();
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        logger(),
        "Exception in result_callback for [%s]: %s",
        action_name_.c_str(),
        e.what());
    } catch (...) {
      RCLCPP_ERROR(
        logger(),
        "Unknown exception in result_callback for [%s]",
        action_name_.c_str());
    }
  };
    //--------------------
    // Check if server is ready
    if(!action_client->action_server_is_ready())
    {
      return onFailure(SERVER_UNREACHABLE);
    }

    future_goal_handle_ = action_client->async_send_goal(goal, goal_options);
    time_goal_sent_ = now();

    return NodeStatus::RUNNING;
  }

  if(status() == NodeStatus::RUNNING)
  {
    std::unique_lock<std::mutex> lock(getMutex(), std::defer_lock);
    if(callback_execution_mode_ != RosCallbackExecutionMode::SharedExecutor)
    {
      lock.lock();
      client_instance_->callback_executor.spin_some();
    }

    // FIRST case: check if the goal request has a timeout.
    // Take a snapshot under the lock so we don't race with goal_response_callback.
    bool goal_received_snapshot;
    {
      std::lock_guard<std::mutex> state_lock(action_state_mutex_);
      goal_received_snapshot = goal_received_;
    }

    if(!goal_received_snapshot)
    {
      if(!future_goal_handle_.valid()) {
        RCLCPP_WARN(
          logger(),
          "[%s] goal_received_ is false but future_goal_handle_ has no state",
          action_name_.c_str());
        return CheckStatus(onFailure(SEND_GOAL_TIMEOUT));
      }

      auto timeout =
          rclcpp::Duration::from_seconds(double(server_timeout_.count()) / 1000);

      bool goal_future_ready = false;
      if(callback_execution_mode_ == RosCallbackExecutionMode::SharedExecutor)
      {
        goal_future_ready =
          future_goal_handle_.wait_for(std::chrono::milliseconds(0)) ==
          std::future_status::ready;
      }
      else
      {
        auto nodelay = std::chrono::milliseconds(0);
        goal_future_ready =
          client_instance_->callback_executor.spin_until_future_complete(
          future_goal_handle_, nodelay) == rclcpp::FutureReturnCode::SUCCESS;
      }
      if(!goal_future_ready)
      {
        if((now() - time_goal_sent_) > timeout)
        {
          return CheckStatus(onFailure(SEND_GOAL_TIMEOUT));
        }
        else
        {
          return NodeStatus::RUNNING;
        }
      }
      else
      {
        auto handle = future_goal_handle_.get();
        future_goal_handle_ = {};

        {
          std::lock_guard<std::mutex> state_lock(action_state_mutex_);
          // goal_response_callback may have already set these; only write if it hasn't.
          if (!goal_received_) {
            goal_handle_ = handle;
            goal_received_ = true;
          }
          // Belt-and-suspenders: promote any pending result that arrived before
          // goal_response_callback fired and that callback didn't already handle.
          if (has_pending_result_ && goal_handle_) {
            if (pending_result_.goal_id == goal_handle_->get_goal_id()) {
              result_ = pending_result_;
              goal_handle_.reset();
            } else {
              RCLCPP_WARN(
                logger(),
                "Discarding pending early result for [%s]: goal_id does not match "
                "accepted goal.",
                action_name_.c_str());
            }
            pending_result_ = {};
            has_pending_result_ = false;
          }
        }

        if(!handle)
        {
          return CheckStatus(onFailure(GOAL_REJECTED_BY_SERVER));
        }
      }
    }

    if(callback_execution_mode_ == RosCallbackExecutionMode::SharedExecutor)
    {
      std::deque<std::shared_ptr<const Feedback>> feedback_queue;
      {
        std::lock_guard<std::mutex> feedback_lock(on_feedback_state_change_mutex_);
        feedback_queue.swap(pending_feedback_);
      }
      for(const auto& feedback : feedback_queue)
      {
        const auto feedback_status = onFeedback(feedback);
        if(feedback_status == NodeStatus::IDLE)
        {
          throw std::logic_error("onFeedback must not return IDLE");
        }
        std::lock_guard<std::mutex> feedback_lock(on_feedback_state_change_mutex_);
        on_feedback_state_change_ = feedback_status;
      }
    }

    // SECOND case: onFeedback requested a stop.
    // Snapshot under its mutex to avoid racing with feedback_callback.
    NodeStatus feedback_state;
    {
      std::lock_guard<std::mutex> feedback_lock(on_feedback_state_change_mutex_);
      feedback_state = on_feedback_state_change_;
    }
    if(feedback_state != NodeStatus::RUNNING)
    {
      cancelGoal();
      return feedback_state;
    }
    // THIRD case: result received.
    // Snapshot under lock so we don't race with result_callback.
    WrappedResult result_snapshot;
    {
      std::lock_guard<std::mutex> state_lock(action_state_mutex_);
      result_snapshot = result_;
    }

    if(result_snapshot.code != rclcpp_action::ResultCode::UNKNOWN)
    {
      if(result_snapshot.code == rclcpp_action::ResultCode::ABORTED)
      {
        return CheckStatus(onFailure(ACTION_ABORTED));
      }
      else if(result_snapshot.code == rclcpp_action::ResultCode::CANCELED)
      {
        return CheckStatus(onFailure(ACTION_CANCELLED));
      }
      else
      {
        return CheckStatus(onResultReceived(result_snapshot));
      }
    }
  }
  return NodeStatus::RUNNING;
}

template <class T>
inline void RosActionNode<T>::halt()
{
  if(status() == BT::NodeStatus::RUNNING)
  {
    cancelGoal();
    onHalt();
  }
}

template <class T>
inline void RosActionNode<T>::cancelGoal()
{
  if (!client_instance_)
  {
    RCLCPP_WARN(logger(), "cancelGoal called for [%s] with no client instance", action_name_.c_str());
    return;
  }

  auto& executor = client_instance_->callback_executor;
  auto& action_client = client_instance_->action_client;

  if(callback_execution_mode_ == RosCallbackExecutionMode::SharedExecutor)
  {
    typename GoalHandle::SharedPtr local_goal_handle;
    {
      std::lock_guard<std::mutex> state_lock(action_state_mutex_);
      cancellation_requested_ = true;
      local_goal_handle = goal_handle_;
    }

    if(!local_goal_handle && future_goal_handle_.valid() &&
       future_goal_handle_.wait_for(std::chrono::milliseconds(0)) ==
       std::future_status::ready)
    {
      local_goal_handle = future_goal_handle_.get();
      future_goal_handle_ = {};
    }

    if(local_goal_handle)
    {
      try {
        action_client->async_cancel_goal(local_goal_handle);
      } catch(const std::exception& e) {
        RCLCPP_WARN(
          logger(), "cancelGoal for [%s] failed: %s", action_name_.c_str(), e.what());
      }
    }
    else if(future_goal_handle_.valid())
    {
      future_goal_handle_ = {};
    }

    return;
  }

  // If we already have a terminal result, there is nothing left to cancel.
  {
    std::lock_guard<std::mutex> state_lock(action_state_mutex_);
    if (result_.code != rclcpp_action::ResultCode::UNKNOWN)
    {
      RCLCPP_DEBUG(
        logger(),
        "cancelGoal skipped for [%s]: terminal result already received",
        action_name_.c_str());

      goal_handle_.reset();
      future_goal_handle_ = {};
      goal_received_ = false;
      pending_result_ = {};
      has_pending_result_ = false;
      return;
    }
  }

  typename GoalHandle::SharedPtr local_goal_handle;

  {
    std::lock_guard<std::mutex> state_lock(action_state_mutex_);
    local_goal_handle = goal_handle_;
  }

  // If the goal has been sent but not yet accepted/rejected, try to resolve that future first.
  if (!local_goal_handle)
  {
    if (future_goal_handle_.valid())
    {
      auto ret = executor.spin_until_future_complete(future_goal_handle_, server_timeout_);

      if (ret == rclcpp::FutureReturnCode::SUCCESS)
      {
        local_goal_handle = future_goal_handle_.get();
        future_goal_handle_ = {};

        if (local_goal_handle)
        {
          std::lock_guard<std::mutex> state_lock(action_state_mutex_);
          goal_handle_ = local_goal_handle;
          goal_received_ = true;
        }
        else
        {
          // Goal rejected. Nothing to cancel.
          RCLCPP_DEBUG(
            logger(),
            "cancelGoal for [%s]: goal was rejected or null handle returned",
            action_name_.c_str());

          future_goal_handle_ = {};
          {
            std::lock_guard<std::mutex> state_lock(action_state_mutex_);
            goal_received_ = false;
            pending_result_ = {};
            has_pending_result_ = false;
          }
          return;
        }
      }
      else
      {
        // Goal acceptance is still unresolved or timed out. Don't crash trying to cancel.
        RCLCPP_WARN(
          logger(),
          "cancelGoal for [%s]: goal handle not ready in time, skipping cancel",
          action_name_.c_str());

        future_goal_handle_ = {};
        {
          std::lock_guard<std::mutex> state_lock(action_state_mutex_);
          goal_received_ = false;
          pending_result_ = {};
          has_pending_result_ = false;
        }
        return;
      }
    }
    else
    {
      RCLCPP_DEBUG(
        logger(),
        "cancelGoal for [%s]: no active goal_handle_ and no pending future",
        action_name_.c_str());
      return;
    }
  }

  // Try cancel first. This is the meaningful halt operation.
  try
  {
    auto future_cancel = action_client->async_cancel_goal(local_goal_handle);

    if (executor.spin_until_future_complete(future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(
        logger(),
        "cancelGoal for [%s]: cancel request did not complete in time",
        action_name_.c_str());
    }
    else
    {
      RCLCPP_DEBUG(logger(), "cancelGoal for [%s]: cancel request completed", action_name_.c_str());
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(
      logger(),
      "cancelGoal for [%s]: async_cancel_goal threw: %s",
      action_name_.c_str(),
      e.what());
  }
  catch (...)
  {
    RCLCPP_WARN(
      logger(),
      "cancelGoal for [%s]: async_cancel_goal threw unknown exception",
      action_name_.c_str());
  }

  // Optionally wait for the terminal result after cancel, but never crash if the
  // action client no longer recognizes the goal handle.
  try
  {
    auto future_result = action_client->async_get_result(local_goal_handle);

    if (executor.spin_until_future_complete(future_result, server_timeout_) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto wrapped_result = future_result.get();

      {
        std::lock_guard<std::mutex> state_lock(action_state_mutex_);
        result_ = wrapped_result;
      }

      RCLCPP_DEBUG(
        logger(),
        "cancelGoal for [%s]: terminal result received after cancel",
        action_name_.c_str());
    }
    else
    {
      RCLCPP_WARN(
        logger(),
        "cancelGoal for [%s]: timed out waiting for result after cancel",
        action_name_.c_str());
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(
      logger(),
      "cancelGoal for [%s]: async_get_result threw: %s",
      action_name_.c_str(),
      e.what());
  }
  catch (...)
  {
    RCLCPP_WARN(
      logger(),
      "cancelGoal for [%s]: async_get_result threw unknown exception",
      action_name_.c_str());
  }

  // Clear local goal state no matter what. The node is being halted.
  {
    std::lock_guard<std::mutex> state_lock(action_state_mutex_);
    goal_handle_.reset();
    goal_received_ = false;
    pending_result_ = {};
    has_pending_result_ = false;
  }

  future_goal_handle_ = {};
}

template <class T>
inline bool RosActionNode<T>::checkActionClient()
{
  bool found =
    client_instance_->action_client->wait_for_action_server(wait_for_server_timeout_);
  if (!found) {
    RCLCPP_ERROR(
      logger(), "%s: Action server with name '%s' is not reachable.", name().c_str(),
      action_name_.c_str());
  }
  return found;
}

}  // namespace BT
