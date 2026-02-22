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

#pragma once

#include "auto_apms_behavior_tree/executor/options.hpp"
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

struct BehaviorNavigatorTreeExecutorArgs
{
  auto_apms_behavior_tree::TreeExecutorNodeOptions executor_options;
  std::string navigator_name;
  rclcpp::Logger * logger;
  std::shared_ptr<Nav2ClientNodeT> client_node;
  nav2_core::FeedbackUtils feedback_utils;
  nav2_core::NavigatorMuxer * plugin_muxer;
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother;
  std::chrono::milliseconds bt_loop_duration;
  std::chrono::milliseconds server_timeout;
  std::chrono::milliseconds wait_for_service_timeout;
  std::chrono::milliseconds cancel_timeout;

  void preconfigureBlackboard(auto_apms_behavior_tree::TreeBlackboard & bb);
};

template <class TreeExecutorNodeT>
class BehaviorNavigatorTemplate : public nav2_core::NavigatorBase
{
public:
  BehaviorNavigatorTemplate(const std::string & navigator_name);
  ~BehaviorNavigatorTemplate() override = default;

  bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node, const std::vector<std::string> & plugin_lib_names,
    const nav2_core::FeedbackUtils & feedback_utils, nav2_core::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  bool on_activate() override;

  bool on_deactivate() override;

  bool on_cleanup() override;

private:
  const std::string navigator_name_;
  rclcpp::Logger logger_;
  Nav2ClientNodeT::SharedPtr client_node_;
  std::shared_ptr<TreeExecutorNodeT> tree_executor_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> spinner_;
  std::thread spin_thread_;
};

template <class TreeExecutorNodeT>
BehaviorNavigatorTemplate<TreeExecutorNodeT>::BehaviorNavigatorTemplate(const std::string & navigator_name)
: NavigatorBase(), navigator_name_(navigator_name), logger_(rclcpp::get_logger(navigator_name))
{
}

template <class TreeExecutorNodeT>
bool BehaviorNavigatorTemplate<TreeExecutorNodeT>::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node, const std::vector<std::string> & /*plugin_lib_names*/,
  const nav2_core::FeedbackUtils & feedback_utils, nav2_core::NavigatorMuxer * plugin_muxer,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  auto node = parent_node.lock();
  if (!node) {
    return false;
  }

  logger_ = node->get_logger().get_child(navigator_name_);

  // Declare configurable parameters on the parent node
  node->declare_parameter(
    navigator_name_ + ".default_build_handler",
    rclcpp::ParameterValue(std::string("auto_apms_behavior_tree::TreeFromResourceBuildHandler")));

  const std::string default_build_handler = node->get_parameter(navigator_name_ + ".default_build_handler").as_string();

  // Configure the TreeExecutorNode with custom action name and default build handler
  auto_apms_behavior_tree::TreeExecutorNodeOptions executor_options(rclcpp::NodeOptions{}.use_global_arguments(false));
  executor_options.setDefaultBuildHandler(default_build_handler);

  // Create a dedicated client node for Nav2 compatibility
  std::string client_node_name = navigator_name_ + "_nav2";
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

  BehaviorNavigatorTreeExecutorArgs args;
  args.executor_options = executor_options;
  args.navigator_name = navigator_name_;
  args.logger = &logger_;
  args.client_node = client_node_;
  args.feedback_utils = feedback_utils;
  args.plugin_muxer = plugin_muxer;
  args.odom_smoother = odom_smoother;
  args.bt_loop_duration =
    std::chrono::milliseconds(10);  // Will be overridden by actual tick rate in TreeExecutorNode constructor
  args.server_timeout = server_timeout;
  args.wait_for_service_timeout = wait_for_service_timeout;
  args.cancel_timeout = cancel_timeout;

  tree_executor_node_ = std::make_shared<TreeExecutorNodeT>(args);

  return true;
}

template <class TreeExecutorNodeT>
bool BehaviorNavigatorTemplate<TreeExecutorNodeT>::on_activate()
{
  spinner_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  spinner_->add_node(tree_executor_node_->get_node_base_interface());
  spinner_->add_node(client_node_->get_node_base_interface());
  spin_thread_ = std::thread([this]() { spinner_->spin(); });

  return true;
}

template <class TreeExecutorNodeT>
bool BehaviorNavigatorTemplate<TreeExecutorNodeT>::on_deactivate()
{
  if (tree_executor_node_ && tree_executor_node_->isBusy()) {
    RCLCPP_INFO(logger_, "Halting running behavior tree execution.");
    tree_executor_node_->setControlCommand(auto_apms_behavior_tree::TreeExecutorBase::ControlCommand::TERMINATE);
  }
  if (spinner_) {
    spinner_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }

  return true;
}

template <class TreeExecutorNodeT>
bool BehaviorNavigatorTemplate<TreeExecutorNodeT>::on_cleanup()
{
  tree_executor_node_.reset();
#ifndef AUTO_APMS_NAV2_ROS_JAZZY
  client_node_->deactivate();
  client_node_->cleanup();
#endif
  client_node_.reset();
  spinner_.reset();

  return true;
}

}  // namespace auto_apms_nav2