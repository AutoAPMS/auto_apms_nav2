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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "auto_apms_behavior_tree/build_handler/build_handler.hpp"
#include "auto_apms_behavior_tree/build_handler/build_handler_loader.hpp"
#include "auto_apms_behavior_tree/executor/executor_base.hpp"
#include "auto_apms_behavior_tree_core/behavior.hpp"
#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_loader.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "auto_apms_interfaces/action/start_tree_executor.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace auto_apms_behavior_tree;

namespace auto_apms_nav2
{

/**
 * @brief Generic navigator plugin that bridges Nav2 and AutoAPMS.
 *
 * This navigator implements the nav2_core::NavigatorBase interface and uses AutoAPMS's
 * TreeExecutorBase class to execute behavior trees. It exposes a StartTreeExecutor action
 * server that accepts AutoAPMS behavior build requests, enabling any behavior registered
 * with AutoAPMS's resource system to be executed within the Nav2 navigation framework.
 *
 * Key features:
 *  - Leverages AutoAPMS's extensible behavior tree node registration system
 *  - Supports AutoAPMS build handlers for pluggable tree construction strategies
 *  - Populates the behavior tree blackboard with Nav2 utilities (TF, frames, odometry)
 *  - Integrates with Nav2's NavigatorMuxer for exclusive navigation access
 *  - Supports Groot2 monitoring via configurable ZMQ publisher
 */
class BehaviorTreeNavigator : public nav2_core::NavigatorBase
{
  inline static const std::string NAVIGATOR_NAME = "auto_apms_navigator";

  /**
   * @brief Inner executor derived from TreeExecutorBase.
   *
   * Provides the behavior tree tick loop and execution state management. A termination
   * callback is forwarded to the outer navigator class so the action goal can be completed.
   */
  class TreeExecutor : public TreeExecutorBase
  {
  public:
    using TreeExecutorBase::TreeExecutorBase;

    using TerminationCallback = std::function<void(ExecutionResult)>;

    /// Register a callback invoked when execution terminates.
    void setTerminationCallback(TerminationCallback callback)
    {
      std::lock_guard<std::mutex> lock(cb_mutex_);
      termination_callback_ = std::move(callback);
    }

  private:
    void onTermination(const ExecutionResult & result) override
    {
      std::lock_guard<std::mutex> lock(cb_mutex_);
      if (termination_callback_) {
        termination_callback_(result);
      }
    }

    std::mutex cb_mutex_;
    TerminationCallback termination_callback_;
  };

public:
  using StartTreeExecutor = auto_apms_interfaces::action::StartTreeExecutor;
  using GoalHandle = rclcpp_action::ServerGoalHandle<StartTreeExecutor>;

  BehaviorTreeNavigator() = default;
  ~BehaviorTreeNavigator() override = default;

  // ---------------------------------------------------------------------------
  // NavigatorBase lifecycle
  // ---------------------------------------------------------------------------

  bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node, const std::vector<std::string> & /*plugin_lib_names*/,
    const nav2_core::FeedbackUtils & feedback_utils, nav2_core::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override
  {
    parent_node_ = parent_node;
    auto node = parent_node.lock();
    if (!node) {
      return false;
    }

    logger_ = node->get_logger();
    clock_ = node->get_clock();
    feedback_utils_ = feedback_utils;
    plugin_muxer_ = plugin_muxer;
    odom_smoother_ = odom_smoother;

    // Declare configurable parameters
    node->declare_parameter(NAVIGATOR_NAME + ".tick_rate", rclcpp::ParameterValue(0.1));
    node->declare_parameter(NAVIGATOR_NAME + ".groot2_port", rclcpp::ParameterValue(-1));
    node->declare_parameter(NAVIGATOR_NAME + ".action_name", rclcpp::ParameterValue("start"));
    node->declare_parameter(NAVIGATOR_NAME + ".default_build_handler", rclcpp::ParameterValue(std::string("")));

    tick_rate_ = node->get_parameter(NAVIGATOR_NAME + ".tick_rate").as_double();
    groot2_port_ = static_cast<int>(node->get_parameter(NAVIGATOR_NAME + ".groot2_port").as_int());
    action_name_ = node->get_parameter(NAVIGATOR_NAME + ".action_name").as_string();
    default_build_handler_ = node->get_parameter(NAVIGATOR_NAME + ".default_build_handler").as_string();

    // Create an internal rclcpp::Node that the TreeExecutorBase uses for timers
    // and ROS context. It inherits the parent node's namespace.
    rclcpp::NodeOptions node_options;
    node_options.use_global_arguments(false);
    ros_node_ = std::make_shared<rclcpp::Node>("_" + NAVIGATOR_NAME, node->get_namespace(), node_options);

    // Instantiate the tree executor
    executor_ = std::make_shared<TreeExecutor>(ros_node_);

    // Set up the behavior tree node plugin loader and build handler loader
    node_loader_ = core::NodeRegistrationLoader::make_shared();
    build_handler_loader_ = std::make_unique<TreeBuildHandlerLoader>();

    // Populate the global blackboard with Nav2 utilities so BT nodes can access them
    setNav2BlackboardDefaults();

    RCLCPP_INFO(logger_, "AutoAPMS BehaviorTreeNavigator configured (action: '%s').", action_name_.c_str());
    return true;
  }

  bool on_activate() override
  {
    // Create the action server on the internal node
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<StartTreeExecutor>(
      ros_node_, action_name_, std::bind(&BehaviorTreeNavigator::handleGoal, this, _1, _2),
      std::bind(&BehaviorTreeNavigator::handleCancel, this, _1),
      std::bind(&BehaviorTreeNavigator::handleAccepted, this, _1));

    // Spin the internal node in a dedicated thread
    spinner_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner_->add_node(ros_node_);
    spin_thread_ = std::thread([this]() { spinner_->spin(); });

    RCLCPP_INFO(logger_, "AutoAPMS BehaviorTreeNavigator activated.");
    return true;
  }

  bool on_deactivate() override
  {
    // Terminate any running execution
    if (executor_ && executor_->isBusy()) {
      RCLCPP_INFO(logger_, "Halting running behavior tree execution.");
      executor_->setControlCommand(TreeExecutorBase::ControlCommand::TERMINATE);
    }

    // Wait for execution thread to finish
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }

    // Tear down the spinner
    if (spinner_) {
      spinner_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    action_server_.reset();
    RCLCPP_INFO(logger_, "AutoAPMS BehaviorTreeNavigator deactivated.");
    return true;
  }

  bool on_cleanup() override
  {
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }

    build_handler_.reset();
    build_handler_loader_.reset();
    node_loader_.reset();
    executor_.reset();
    spinner_.reset();
    ros_node_.reset();

    RCLCPP_INFO(logger_, "AutoAPMS BehaviorTreeNavigator cleaned up.");
    return true;
  }

private:
  // ---------------------------------------------------------------------------
  // Action server callbacks
  // ---------------------------------------------------------------------------

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const StartTreeExecutor::Goal> goal)
  {
    RCLCPP_INFO(
      logger_, "Received behavior tree execution request (build_request: '%s', build_handler: '%s').",
      goal->build_request.c_str(), goal->build_handler.c_str());

    // Reject if another navigator is already active
    if (plugin_muxer_->isNavigating()) {
      RCLCPP_WARN(logger_, "Another navigator is already active — rejecting request.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Reject if the executor is already busy
    if (executor_->isBusy()) {
      RCLCPP_WARN(logger_, "Executor is already busy — rejecting request.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    RCLCPP_INFO(logger_, "Received request to cancel behavior tree execution.");
    if (executor_->isBusy()) {
      executor_->setControlCommand(TreeExecutorBase::ControlCommand::TERMINATE);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(std::shared_ptr<GoalHandle> goal_handle)
  {
    // Wait for any previous execution thread to complete
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }

    // Execute the goal in a separate thread so we don't block the executor spin loop
    execution_thread_ = std::thread(&BehaviorTreeNavigator::executeGoal, this, goal_handle);
  }

  // ---------------------------------------------------------------------------
  // Goal execution
  // ---------------------------------------------------------------------------

  void executeGoal(std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<StartTreeExecutor::Result>();

    // Lock the navigator muxer
    plugin_muxer_->startNavigating(NAVIGATOR_NAME);

    try {
      // --- Build handler selection ---
      std::string build_handler_name = goal->build_handler;
      if (build_handler_name.empty()) {
        build_handler_name = default_build_handler_;
      }

      if (!build_handler_name.empty() && build_handler_name != current_build_handler_name_) {
        loadBuildHandler(build_handler_name);
      }

      // --- Parse node manifest from goal if provided ---
      core::NodeManifest node_manifest;
      if (!goal->node_manifest.empty()) {
        node_manifest = core::NodeManifest::decode(goal->node_manifest);
      }

      // --- Optionally clear and re-populate the global blackboard ---
      if (goal->clear_blackboard) {
        executor_->clearGlobalBlackboard();
        setNav2BlackboardDefaults();
      }

      // --- Create the tree constructor callback ---
      TreeConstructor tree_constructor = makeTreeConstructor(goal->build_request, goal->entry_point, node_manifest);

      // --- Start execution ---
      auto execution_future = executor_->startExecution(tree_constructor, tick_rate_, groot2_port_);

      // --- Monitor execution, publish feedback, handle cancellation ---
      const auto feedback_interval = std::chrono::duration<double>(tick_rate_);
      while (rclcpp::ok()) {
        // Check for cancellation
        if (goal_handle->is_canceling()) {
          executor_->setControlCommand(TreeExecutorBase::ControlCommand::TERMINATE);

          // Wait for executor to actually stop
          execution_future.wait();

          result->message = "Navigation cancelled.";
          result->tree_result = StartTreeExecutor::Result::TREE_RESULT_NOT_SET;
          goal_handle->canceled(result);
          plugin_muxer_->stopNavigating(NAVIGATOR_NAME);
          return;
        }

        // Check if execution finished
        if (execution_future.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) {
          break;
        }

        // Publish feedback
        auto feedback = std::make_shared<StartTreeExecutor::Feedback>();
        feedback->execution_state_str = toStr(executor_->getExecutionState());
        feedback->running_tree_identity = executor_->getTreeName();
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(feedback_interval);
      }

      // --- Process final result ---
      auto exec_result = execution_future.get();
      switch (exec_result) {
        case TreeExecutorBase::ExecutionResult::TREE_SUCCEEDED:
          result->tree_result = StartTreeExecutor::Result::TREE_RESULT_SUCCESS;
          result->message = "Behavior tree completed successfully.";
          RCLCPP_INFO(logger_, "%s", result->message.c_str());
          goal_handle->succeed(result);
          break;
        case TreeExecutorBase::ExecutionResult::TREE_FAILED:
          result->tree_result = StartTreeExecutor::Result::TREE_RESULT_FAILURE;
          result->message = "Behavior tree completed with failure.";
          RCLCPP_WARN(logger_, "%s", result->message.c_str());
          goal_handle->abort(result);
          break;
        case TreeExecutorBase::ExecutionResult::TERMINATED_PREMATURELY:
          result->tree_result = StartTreeExecutor::Result::TREE_RESULT_NOT_SET;
          result->message = "Behavior tree terminated prematurely.";
          RCLCPP_WARN(logger_, "%s", result->message.c_str());
          goal_handle->abort(result);
          break;
        default:
          result->tree_result = StartTreeExecutor::Result::TREE_RESULT_NOT_SET;
          result->message = "Behavior tree execution error.";
          RCLCPP_ERROR(logger_, "%s", result->message.c_str());
          goal_handle->abort(result);
          break;
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Error during behavior tree execution: %s", e.what());
      result->tree_result = StartTreeExecutor::Result::TREE_RESULT_NOT_SET;
      result->message = std::string("Exception: ") + e.what();

      if (goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    }

    plugin_muxer_->stopNavigating(NAVIGATOR_NAME);
  }

  // ---------------------------------------------------------------------------
  // Tree construction helpers
  // ---------------------------------------------------------------------------

  /**
   * @brief Populate the global blackboard with Nav2-specific entries.
   *
   * This makes TF buffer, frame IDs, and odometry data available to behavior tree
   * nodes via the global blackboard (accessible with the '@' prefix).
   */
  void setNav2BlackboardDefaults()
  {
    auto bb = executor_->getGlobalBlackboardPtr();
    bb->set("tf_buffer", feedback_utils_.tf);
    bb->set("global_frame", feedback_utils_.global_frame);
    bb->set("robot_frame", feedback_utils_.robot_frame);
    bb->set("transform_tolerance", feedback_utils_.transform_tolerance);
    bb->set("odom_smoother", odom_smoother_);
  }

  /**
   * @brief Load a build handler plugin by its fully qualified class name.
   */
  void loadBuildHandler(const std::string & name)
  {
    if (name.empty()) {
      build_handler_.reset();
      current_build_handler_name_.clear();
      return;
    }

    RCLCPP_INFO(logger_, "Loading build handler: '%s'.", name.c_str());
    auto factory = build_handler_loader_->createUniqueInstance(name);
    build_handler_ = factory->makeUnique(ros_node_, node_loader_);
    current_build_handler_name_ = name;
  }

  /**
   * @brief Create a TreeConstructor callback that builds a behavior tree.
   *
   * If a build handler is active, it is used to construct the tree. Otherwise, the
   * build_request is interpreted as either a behavior resource identity or a file path,
   * providing a convenient default for simple use cases.
   */
  TreeConstructor makeTreeConstructor(
    const std::string & build_request, const std::string & entry_point, const core::NodeManifest & node_manifest)
  {
    // Validate the request with the build handler (if one is loaded)
    if (build_handler_) {
      if (!build_handler_->setBuildRequest(build_request, entry_point, node_manifest)) {
        throw std::runtime_error("Build handler '" + current_build_handler_name_ + "' rejected the build request.");
      }
    }

    return [this, build_request, entry_point, node_manifest](TreeBlackboardSharedPtr global_bb) -> Tree {
      auto builder = std::make_unique<core::TreeBuilder>(
        ros_node_, executor_->getTreeNodeWaitablesCallbackGroupPtr(), executor_->getTreeNodeWaitablesExecutorPtr(),
        node_loader_);

      // Register nodes from the provided manifest
      if (!node_manifest.empty()) {
        builder->registerNodes(node_manifest);
      }

      if (build_handler_) {
        // --- Build handler path: delegate tree construction to the plugin ---
        auto root_tree = build_handler_->buildTree(*builder, *global_bb);
        return builder->instantiate(root_tree.getName(), global_bb);
      }

      // --- Default path: interpret build_request as a resource or file ---
      // First, try to load as a behavior resource identity
      try {
        core::BehaviorResource resource(build_request);

        // Register additional nodes from the behavior resource's manifest
        const auto & res_manifest = resource.getNodeManifest();
        if (!res_manifest.empty()) {
          builder->registerNodes(res_manifest);
        }

        // Use the build request (typically a file path or XML) from the resource
        const auto & res_build_request = resource.getBuildRequest();
        builder->mergeFile(res_build_request);

        // Determine the entry point
        std::string tree_name = entry_point;
        if (tree_name.empty()) {
          tree_name = resource.getEntryPoint();
        }

        if (!tree_name.empty()) {
          return builder->instantiate(tree_name, global_bb);
        }
        return builder->instantiate(global_bb);

      } catch (const std::exception &) {
        // Not a behavior resource — try as a tree resource identity
        try {
          core::TreeResource tree_resource(build_request);
          builder->mergeResource(tree_resource);

          // Register nodes from the tree resource's associated manifest
          const auto & res_manifest = tree_resource.getNodeManifest();
          if (!res_manifest.empty()) {
            builder->registerNodes(res_manifest);
          }

          std::string tree_name = entry_point;
          if (tree_name.empty() && tree_resource.hasRootTreeName()) {
            tree_name = tree_resource.getRootTreeName();
          }

          if (!tree_name.empty()) {
            return builder->instantiate(tree_name, global_bb);
          }
          return builder->instantiate(global_bb);

        } catch (const std::exception &) {
          // Last resort: interpret as a file path
          builder->mergeFile(build_request);

          if (!entry_point.empty()) {
            return builder->instantiate(entry_point, global_bb);
          }
          return builder->instantiate(global_bb);
        }
      }
    };
  }

  // ---------------------------------------------------------------------------
  // Member variables
  // ---------------------------------------------------------------------------

  // Nav2 state
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  nav2_core::NavigatorMuxer * plugin_muxer_{nullptr};
  nav2_core::FeedbackUtils feedback_utils_;
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
  rclcpp::Logger logger_{rclcpp::get_logger(NAVIGATOR_NAME)};
  rclcpp::Clock::SharedPtr clock_;

  // Configuration
  double tick_rate_{0.1};
  int groot2_port_{-1};
  std::string action_name_;
  std::string default_build_handler_;

  // Internal ROS 2 node and executor for spinning the tree tick timer
  rclcpp::Node::SharedPtr ros_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> spinner_;
  std::thread spin_thread_;

  // AutoAPMS tree executor
  std::shared_ptr<TreeExecutor> executor_;

  // AutoAPMS tree construction components
  core::NodeRegistrationLoader::SharedPtr node_loader_;
  TreeBuildHandlerLoader::UniquePtr build_handler_loader_;
  TreeBuildHandler::UniquePtr build_handler_;
  std::string current_build_handler_name_;

  // Action server and execution threading
  rclcpp_action::Server<StartTreeExecutor>::SharedPtr action_server_;
  std::thread execution_thread_;
};

}  // namespace auto_apms_nav2

PLUGINLIB_EXPORT_CLASS(auto_apms_nav2::BehaviorTreeNavigator, nav2_core::NavigatorBase)