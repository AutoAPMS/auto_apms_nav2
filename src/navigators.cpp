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

#include "auto_apms_behavior_tree/executor/action_based_executor.hpp"
#include "auto_apms_behavior_tree/executor/executor_node.hpp"
#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_nav2/behavior_navigator_template.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace auto_apms_nav2
{

void BehaviorNavigatorTreeExecutorArgs::preconfigureBlackboard(auto_apms_behavior_tree::TreeBlackboard & bb)
{
  bb.set("node", client_node);
  bb.set("tf_buffer", feedback_utils.tf);
  bb.set("initial_pose_received", false);
  bb.set("number_recoveries", 0);
  bb.set("transform_tolerance", feedback_utils.transform_tolerance);
  bb.set("odom_smoother", odom_smoother);
  bb.set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration);
  bb.set<std::chrono::milliseconds>("server_timeout", server_timeout);
  bb.set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout);
  bb.set<std::chrono::milliseconds>("cancel_timeout", cancel_timeout);
}

class GenericBehaviorExecutorNode : public auto_apms_behavior_tree::TreeExecutorNode
{
public:
  GenericBehaviorExecutorNode(BehaviorNavigatorTreeExecutorArgs args)
  : TreeExecutorNode(args.navigator_name, args.executor_options), args_(args)
  {
    double tick_rate = this->getExecutorParameters().tick_rate;
    args_.bt_loop_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(tick_rate));
  }

private:
  bool shouldAcceptGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TriggerGoal> goal_ptr) override
  {
    if (args_.plugin_muxer->isNavigating()) {
      RCLCPP_WARN(
        *args_.logger, "Goal %s was REJECTED: Another navigator is already active.",
        rclcpp_action::to_string(uuid).c_str());
      return false;
    }
    return TreeExecutorNode::shouldAcceptGoal(uuid, goal_ptr);
  }

  void onAcceptedGoal(std::shared_ptr<TriggerActionContext::GoalHandle> goal_handle_ptr) override
  {
    args_.plugin_muxer->startNavigating(args_.navigator_name);
    TreeExecutorNode::onAcceptedGoal(goal_handle_ptr);
  }

  void onGoalExecutionTermination(const ExecutionResult & result, TriggerActionContext & context) override
  {
    args_.plugin_muxer->stopNavigating(args_.navigator_name);
    TreeExecutorNode::onGoalExecutionTermination(result, context);
  }

  void preBuild(
    auto_apms_behavior_tree::core::TreeBuilder & /*builder*/, const std::string & /*build_request*/,
    const std::string & /*entry_point*/, const auto_apms_behavior_tree::core::NodeManifest & /*node_manifest*/,
    auto_apms_behavior_tree::TreeBlackboard & bb) override
  {
    // Modify blackboard to add Nav2-specific utilities for use in behavior trees
    args_.preconfigureBlackboard(bb);

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
      args_.preconfigureBlackboard(*subtree->blackboard);
    }
  }

  BehaviorNavigatorTreeExecutorArgs args_;
};

class NavigateToPoseExecutorNode
: public auto_apms_behavior_tree::ActionBasedTreeExecutor<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToPoseExecutorNode(BehaviorNavigatorTreeExecutorArgs args)
  : ActionBasedTreeExecutor(args.navigator_name, "navigate_to_pose", args.executor_options), args_(args)
  {
    double tick_rate = this->getExecutorParameters().tick_rate;
    args_.bt_loop_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(tick_rate));
  }

private:
  auto_apms_behavior_tree::TreeConstructor getTreeConstructorFromGoal(
    std::shared_ptr<const TriggerGoal> goal_ptr) override
  {
    return [this, pose = goal_ptr->pose](auto_apms_behavior_tree::TreeBlackboardSharedPtr bb_ptr) {
      auto_apms_behavior_tree::core::TreeBuilder::SharedPtr builder = this->createTreeBuilder();
      auto tree_ele =
        builder->newTreeFromResource("auto_apms_nav2::navigate_to_pose_w_replanning_and_recovery::MainTree");

      // Set the goal pose from the action goal
      bb_ptr->set("goal", pose);

      // Create the tree
      BT::Tree tree = builder->instantiate(tree_ele, bb_ptr);

      // Forward blackboard preconfiguration to all subtrees as well since some Nav2 BT nodes expect certain blackboard
      // entries to be present on their local blackboard
      for (auto & subtree : tree.subtrees) {
        args_.preconfigureBlackboard(*subtree->blackboard);
      }
      return tree;
    };
  }

  bool shouldAcceptGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TriggerGoal> goal_ptr) override
  {
    if (args_.plugin_muxer->isNavigating()) {
      RCLCPP_WARN(
        *args_.logger, "Goal %s was REJECTED: Another navigator is already active.",
        rclcpp_action::to_string(uuid).c_str());
      return false;
    }
    return ActionBasedTreeExecutor::shouldAcceptGoal(uuid, goal_ptr);
  }

  void onAcceptedGoal(std::shared_ptr<TriggerActionContext::GoalHandle> goal_handle_ptr) override
  {
    args_.plugin_muxer->startNavigating(args_.navigator_name);
    ActionBasedTreeExecutor::onAcceptedGoal(goal_handle_ptr);
  }

  void onGoalExecutionTermination(const ExecutionResult & result, TriggerActionContext & context) override
  {
    args_.plugin_muxer->stopNavigating(args_.navigator_name);
    ActionBasedTreeExecutor::onGoalExecutionTermination(result, context);
  }

  BehaviorNavigatorTreeExecutorArgs args_;
};

/**
 * @brief Navigator plugin for executing generic behaviors.
 *
 * This navigator implements the nav2_core::NavigatorBase interface and delegates behavior tree execution to a
 * GenericBehaviorExecutorNode — a TreeExecutorNode subclass that integrates with Nav2's NavigatorMuxer for exclusive
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
class GenericBehaviorNavigator : public BehaviorNavigatorTemplate<GenericBehaviorExecutorNode>
{
public:
  GenericBehaviorNavigator() : BehaviorNavigatorTemplate("auto_apms_generic_behavior_navigator") {};
};

/**
 * @brief Navigator plugin for executing NavigateToPose behavior trees.
 *
 * This navigator is a specialized version of the GenericBehaviorNavigator that is preconfigured to execute
 * NavigateToPose behavior trees. It uses a NavigateToPoseExecutorNode which implements the logic to build and execute
 * a NavigateToPose-specific behavior tree based on the incoming action goal.
 *
 * The expected behavior tree should be designed to read the "goal" blackboard entry for the target pose and utilize
 * Nav2 BT nodes to perform navigation, replanning, and recovery as needed.
 */
class NavigateToPoseNavigator : public BehaviorNavigatorTemplate<NavigateToPoseExecutorNode>
{
public:
  NavigateToPoseNavigator() : BehaviorNavigatorTemplate("auto_apms_navigate_to_pose_navigator") {};
};

}  // namespace auto_apms_nav2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(auto_apms_nav2::GenericBehaviorNavigator, nav2_core::NavigatorBase)
PLUGINLIB_EXPORT_CLASS(auto_apms_nav2::NavigateToPoseNavigator, nav2_core::NavigatorBase)