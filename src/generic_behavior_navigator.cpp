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

// Jazzy: Nav2 BT nodes expect rclcpp::Node
// Kilted+: Nav2 BT nodes expect nav2::LifecycleNode from nav2_ros_common
#ifdef AUTO_APMS_NAV2_ROS_JAZZY
#include "nav2_util/node_utils.hpp"
using Nav2ClientNodeT = rclcpp::Node;
#else
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
using Nav2ClientNodeT = nav2::LifecycleNode;
#endif

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
      auto_apms_behavior_tree::TreeExecutorNodeOptions options, const rclcpp::Logger & logger,
      Nav2ClientNodeT::SharedPtr client_node, const nav2_core::FeedbackUtils & feedback_utils,
      nav2_core::NavigatorMuxer * muxer, std::shared_ptr<nav2_util::OdomSmoother> odom_smoother,
      std::chrono::milliseconds server_timeout, std::chrono::milliseconds wait_for_service_timeout,
      std::chrono::milliseconds cancel_timeout)
    : TreeExecutorNode(NAVIGATOR_NAME, options),
      logger_(logger),
      client_node_(client_node),
      feedback_utils_(feedback_utils),
      plugin_muxer_(muxer),
      odom_smoother_(odom_smoother),
      server_timeout_(server_timeout),
      wait_for_service_timeout_(wait_for_service_timeout),
      cancel_timeout_(cancel_timeout)
    {
    }

  private:
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

    void preBuild(
      auto_apms_behavior_tree::core::TreeBuilder & /*builder*/, const std::string & /*build_request*/,
      const std::string & /*entry_point*/, const auto_apms_behavior_tree::core::NodeManifest & /*node_manifest*/,
      auto_apms_behavior_tree::TreeBlackboard & bb) override
    {
      // Modify blackboard to add Nav2-specific utilities for use in behavior trees
      preconfigureBlackboard(bb);

      // Testing: Set predefined goal pose
      // geometry_msgs::msg::PoseStamped goal_pose;
      // goal_pose.header.frame_id = "map";
      // goal_pose.pose.position.x = 0.6151242852210999;
      // goal_pose.pose.position.y = 0.041284047067165375;
      // goal_pose.pose.position.z = 0.00250244140625;
      // bb.set("goal", goal_pose);
    }

    void postBuild(auto_apms_behavior_tree::Tree & tree) override
    {
      // Forward blackboard preconfiguration to all subtrees as well since some Nav2 BT nodes expect certain blackboard
      // entries to be present on their local blackboard
      for (auto & subtree : tree.subtrees) {
        preconfigureBlackboard(*subtree->blackboard);
      }
    }

    void preconfigureBlackboard(auto_apms_behavior_tree::TreeBlackboard & bb)
    {
      bb.set("node", client_node_);
      bb.set("tf_buffer", feedback_utils_.tf);
      bb.set("initial_pose_received", false);
      bb.set("number_recoveries", 0);
      bb.set("transform_tolerance", feedback_utils_.transform_tolerance);
      bb.set("odom_smoother", odom_smoother_);
      bb.set<std::chrono::milliseconds>(
        "bt_loop_duration", std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::duration<double>(getExecutorParameters().tick_rate)));
      bb.set<std::chrono::milliseconds>("server_timeout", server_timeout_);
      bb.set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout_);
      bb.set<std::chrono::milliseconds>("cancel_timeout", cancel_timeout_);
    }

    rclcpp::Logger logger_;
    Nav2ClientNodeT::SharedPtr client_node_;
    nav2_core::FeedbackUtils feedback_utils_;
    nav2_core::NavigatorMuxer * plugin_muxer_{nullptr};
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
    std::string navigator_name_;
    std::chrono::milliseconds server_timeout_;
    std::chrono::milliseconds wait_for_service_timeout_;
    std::chrono::milliseconds cancel_timeout_;
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
    auto_apms_behavior_tree::TreeExecutorNodeOptions executor_options(
      rclcpp::NodeOptions{}.use_global_arguments(false));
    executor_options.setDefaultBuildHandler(default_build_handler);

    // Create a dedicated client node for Nav2 compatibility
    std::string client_node_name = NAVIGATOR_NAME + "_nav2";
    std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
    // Use suffix '_rclcpp_node' to keep parameter file consistency #1773
    auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "-r",
       std::string("__node:=") + std::string(node->get_name()) + "_" + client_node_name + "_rclcpp_node", "-p",
       "use_sim_time:=" + std::string(node->get_parameter("use_sim_time").as_bool() ? "true" : "false"), "--"});
    client_node_ = std::make_shared<Nav2ClientNodeT>("_", options);
#ifndef AUTO_APMS_NAV2_ROS_JAZZY
    client_node_->configure();
    client_node_->activate();

    nav2::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue(std::string("map")));
    nav2::declare_parameter_if_not_declared(node, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
    nav2::declare_parameter_if_not_declared(node, "transform_tolerance", rclcpp::ParameterValue(0.1));
    nav2::declare_parameter_if_not_declared(node, "default_server_timeout", rclcpp::ParameterValue(20));
    nav2::declare_parameter_if_not_declared(node, "wait_for_service_timeout", rclcpp::ParameterValue(3000));
    nav2::declare_parameter_if_not_declared(node, "default_cancel_timeout", rclcpp::ParameterValue(50));
#else
    nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue(std::string("map")));
    nav2_util::declare_parameter_if_not_declared(
      node, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
    nav2_util::declare_parameter_if_not_declared(node, "transform_tolerance", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(node, "default_server_timeout", rclcpp::ParameterValue(20));
    nav2_util::declare_parameter_if_not_declared(node, "wait_for_service_timeout", rclcpp::ParameterValue(3000));
    nav2_util::declare_parameter_if_not_declared(node, "default_cancel_timeout", rclcpp::ParameterValue(20));
#endif
    rclcpp::copy_all_parameter_values(node, client_node_);

    // Read timeout parameters for blackboard population
    const auto server_timeout = std::chrono::milliseconds(node->get_parameter("default_server_timeout").as_int());
    const auto wait_for_service_timeout =
      std::chrono::milliseconds(node->get_parameter("wait_for_service_timeout").as_int());
    const auto cancel_timeout = std::chrono::milliseconds(node->get_parameter("default_cancel_timeout").as_int());

    executor_node_ = std::make_shared<InternalTreeExecutorNode>(
      executor_options, logger_, client_node_, feedback_utils, plugin_muxer, odom_smoother, server_timeout,
      wait_for_service_timeout, cancel_timeout);

    return true;
  }

  bool on_activate() override
  {
    spinner_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner_->add_node(executor_node_->get_node_base_interface());
    spinner_->add_node(client_node_->get_node_base_interface());
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
#ifndef AUTO_APMS_NAV2_ROS_JAZZY
    client_node_->deactivate();
    client_node_->cleanup();
#endif
    client_node_.reset();
    spinner_.reset();

    return true;
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger(NAVIGATOR_NAME)};
  Nav2ClientNodeT::SharedPtr client_node_;
  std::shared_ptr<InternalTreeExecutorNode> executor_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> spinner_;
  std::thread spin_thread_;
};

}  // namespace auto_apms_nav2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(auto_apms_nav2::GenericBehaviorNavigator, nav2_core::NavigatorBase)