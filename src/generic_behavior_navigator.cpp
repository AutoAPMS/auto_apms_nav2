// Copyright 2026 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_apms_behavior_tree/executor/executor_node.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_nav2
{

/**
 * @brief Navigator plugin for executing generic behaviors.
 *
 * This navigator implements the nav2_core::NavigatorBase interface and delegates behavior tree execution to a
 * InternalTreeExecutorNode — a TreeExecutorNode subclass that integrates with Nav2's NavigatorMuxer for exclusive
 * navigation access.
 *
 * The built-in StartTreeExecutor action server of TreeExecutorNode handles the full action lifecycle (goal
 * validation, feedback publishing, cancellation, result reporting). The navigator merely manages the executor
 * node's lifecycle and configures it with Nav2-specific blackboard entries.
 *
 * Key features:
 *  - Full AutoAPMS executor capabilities (build handlers, parameters, Groot2 monitoring)
 *  - Leverages AutoAPMS's extensible behavior tree node registration system
 *  - Populates the behavior tree blackboard with Nav2 utilities (TF, frames, odometry)
 *  - Integrates with Nav2's NavigatorMuxer for exclusive navigation access
 */
class GenericBehaviorNavigator : public nav2_core::NavigatorBase
{
  inline static const std::string NAVIGATOR_NAME = "auto_apms_generic_behavior_navigator";

  /**
   * @brief TreeExecutorNode subclass that integrates with Nav2's NavigatorMuxer.
   *
   * Overrides the goal validation hook to reject requests when another navigator is active,
   * and manages the muxer lock/unlock around execution via onAcceptedStartGoal/onTermination.
   */
  class InternalTreeExecutorNode : public auto_apms_behavior_tree::TreeExecutorNode
  {
  public:
    InternalTreeExecutorNode(
      auto_apms_behavior_tree::TreeExecutorNodeOptions options, const rclcpp::Logger & logger, const nav2_core::FeedbackUtils & feedback_utils,
      nav2_core::NavigatorMuxer * muxer, std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
    : TreeExecutorNode(NAVIGATOR_NAME, options),
      logger_(logger),
      feedback_utils_(feedback_utils),
      plugin_muxer_(muxer),
      odom_smoother_(odom_smoother)
    {
      // Populate the global blackboard with Nav2 utilities so BT nodes can access them
      setNav2BlackboardDefaults();
    }

  protected:
    bool clearGlobalBlackboard() override
    {
      if (!TreeExecutorNode::clearGlobalBlackboard()) {
        return false;
      }
      setNav2BlackboardDefaults();
      return true;
    }

    bool shouldAcceptStartGoal(
      const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StartActionContext::Goal> goal_ptr) override
    {
      if (plugin_muxer_->isNavigating()) {
        RCLCPP_WARN(
          logger_, "Goal %s was REJECTED: Another navigator is already active.",
          rclcpp_action::to_string(uuid).c_str());
        return false;
      }
      return TreeExecutorNode::shouldAcceptStartGoal(uuid, goal_ptr);
    }

    void onAcceptedStartGoal(std::shared_ptr<StartActionContext::GoalHandle> goal_handle_ptr) override
    {
      plugin_muxer_->startNavigating(navigator_name_);
      TreeExecutorNode::onAcceptedStartGoal(goal_handle_ptr);
    }

    void onTermination(const ExecutionResult & result) override
    {
      plugin_muxer_->stopNavigating(navigator_name_);
      TreeExecutorNode::onTermination(result);
    }

  private:
    void setNav2BlackboardDefaults()
    {
      auto bb = getGlobalBlackboardPtr();
      bb->set("tf_buffer", feedback_utils_.tf);
      bb->set("global_frame", feedback_utils_.global_frame);
      bb->set("robot_frame", feedback_utils_.robot_frame);
      bb->set("transform_tolerance", feedback_utils_.transform_tolerance);
      bb->set("odom_smoother", odom_smoother_);
    }

    rclcpp::Logger logger_;
    nav2_core::FeedbackUtils feedback_utils_;
    nav2_core::NavigatorMuxer * plugin_muxer_{nullptr};
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
    std::string navigator_name_;
  };

public:
  GenericBehaviorNavigator() = default;
  ~GenericBehaviorNavigator() override = default;

  bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node, const std::vector<std::string> & /*plugin_lib_names*/,
    const nav2_core::FeedbackUtils & feedback_utils, nav2_core::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override
  {
    auto node = parent_node.lock();
    if (!node) {
      return false;
    }

    logger_ = node->get_logger().get_child(NAVIGATOR_NAME);

    // Declare configurable parameters on the parent node
    node->declare_parameter(
      NAVIGATOR_NAME + ".default_build_handler",
      rclcpp::ParameterValue(std::string("auto_apms_behavior_tree::TreeFromResourceBuildHandler")));

    const std::string default_build_handler =
      node->get_parameter(NAVIGATOR_NAME + ".default_build_handler").as_string();

    // Configure the TreeExecutorNode with custom action name and default build handler
    auto_apms_behavior_tree::TreeExecutorNodeOptions options(rclcpp::NodeOptions{}.use_global_arguments(false));
    options.setDefaultBuildHandler(default_build_handler);

    executor_node_ =
      std::make_shared<InternalTreeExecutorNode>(options, logger_, feedback_utils, plugin_muxer, odom_smoother);

    return true;
  }

  bool on_activate() override
  {
    spinner_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner_->add_node(executor_node_->get_node_base_interface());
    spin_thread_ = std::thread([this]() { spinner_->spin(); });

    return true;
  }

  bool on_deactivate() override
  {
    if (executor_node_ && executor_node_->isBusy()) {
      RCLCPP_INFO(logger_, "Halting running behavior tree execution.");
      executor_node_->setControlCommand(InternalTreeExecutorNode::ControlCommand::TERMINATE);
    }
    if (spinner_) {
      spinner_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    return true;
  }

  bool on_cleanup() override
  {
    executor_node_.reset();
    spinner_.reset();

    return true;
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger(NAVIGATOR_NAME)};
  std::shared_ptr<InternalTreeExecutorNode> executor_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> spinner_;
  std::thread spin_thread_;
};

}  // namespace auto_apms_nav2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(auto_apms_nav2::GenericBehaviorNavigator, nav2_core::NavigatorBase)