cmake_minimum_required(VERSION 3.12)
project(auto_apms_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)

# Kilted and later: Nav2 BT nodes expect nav2::LifecycleNode (from nav2_ros_common)
# Jazzy: Nav2 BT nodes expect rclcpp::Node
if(NOT "$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  find_package(nav2_ros_common REQUIRED)
endif()

# Export library
add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} INTERFACE
  auto_apms_behavior_tree::auto_apms_behavior_tree
)

# =============================================================================
# Navigator plugin
# =============================================================================

add_library(${PROJECT_NAME}_navigator SHARED
  src/generic_behavior_navigator.cpp
)

# Define the ROS distro as a compile-time constant for runtime logging or feature detection
target_compile_definitions(${PROJECT_NAME}_navigator PRIVATE AUTO_APMS_NAV2_ROS_DISTRO="$ENV{ROS_DISTRO}")

# Jazzy-specific behavior (rclcpp::Node client, no cancel_timeout)
if("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  target_compile_definitions(${PROJECT_NAME}_navigator PRIVATE AUTO_APMS_NAV2_ROS_JAZZY)
endif()

target_link_libraries(${PROJECT_NAME}_navigator PRIVATE
  pluginlib::pluginlib
  rclcpp::rclcpp
  auto_apms_behavior_tree::auto_apms_behavior_tree
)
if(TARGET nav2_core::nav2_core)
  target_link_libraries(${PROJECT_NAME}_navigator PRIVATE
    nav2_core::nav2_core
  )
else()
  target_include_directories(${PROJECT_NAME}_navigator PRIVATE
    ${nav2_core_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}_navigator PRIVATE
    ${nav2_core_LIBRARIES}
  )
endif()
if(NOT "$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  target_link_libraries(${PROJECT_NAME}_navigator PRIVATE nav2_ros_common::nav2_ros_common)
endif()

# =============================================================================
# Nav2 BT node adapter for AutoAPMS registration
# =============================================================================
# Per-library adapter classes generated at configure time, each inheriting from
# NodeRegistrationFromLibraryAdapter and loading exactly one Nav2 BT plugin
# shared library.

# --- Step 1: Compile the generator tool ---
set(_generate_nav2_node_registration_adapter_src "${CMAKE_CURRENT_SOURCE_DIR}/src/cli/generate_nav2_node_registration_adapter.cpp")
set(_generate_nav2_node_registration_adapter_bin "${CMAKE_CURRENT_BINARY_DIR}/generate_nav2_node_registration_adapter")

try_compile(
  _generate_nav2_node_registration_adapter_compile_result
  "${CMAKE_CURRENT_BINARY_DIR}/generate_nav2_node_registration_adapter_build"
  "${_generate_nav2_node_registration_adapter_src}"
  CMAKE_FLAGS
    "-DINCLUDE_DIRECTORIES=${nav2_behavior_tree_INCLUDE_DIRS}"
  LINK_LIBRARIES behaviortree_cpp::behaviortree_cpp
  COPY_FILE "${_generate_nav2_node_registration_adapter_bin}"
  OUTPUT_VARIABLE _generate_nav2_node_registration_adapter_compile_output
)
if(NOT _generate_nav2_node_registration_adapter_compile_result)
  message(FATAL_ERROR
    "Failed to compile generate_nav2_node_registration_adapter:\n${_generate_nav2_node_registration_adapter_compile_output}")
endif()

# --- Step 2: Run the generator to produce the C++ source and manifest ---
set(_nav2_generated_cpp "${CMAKE_CURRENT_BINARY_DIR}/nav2_node_registration_from_library_generated.cpp")
set(_nav2_generated_manifest "${CMAKE_CURRENT_BINARY_DIR}/nav2_node_manifest.yaml")
set(_nav2_ns "${PROJECT_NAME}")

execute_process(
  COMMAND "${_generate_nav2_node_registration_adapter_bin}" "${_nav2_ns}" "${_nav2_generated_cpp}" "${_nav2_generated_manifest}"
  OUTPUT_VARIABLE _nav2_class_names_raw
  ERROR_VARIABLE _generate_nav2_node_registration_adapter_stderr
  RESULT_VARIABLE _generate_nav2_node_registration_adapter_result
)
if(NOT _generate_nav2_node_registration_adapter_result EQUAL 0)
  message(FATAL_ERROR
    "generate_nav2_node_registration_adapter failed (exit ${_generate_nav2_node_registration_adapter_result}):\n${_generate_nav2_node_registration_adapter_stderr}")
endif()
message(STATUS "${_generate_nav2_node_registration_adapter_stderr}")

# Parse fully-qualified class names from stdout (one per line).
string(STRIP "${_nav2_class_names_raw}" _nav2_class_names_raw)
string(REPLACE "\n" ";" _nav2_class_names "${_nav2_class_names_raw}")

# --- Step 3: Create a plugin library that registers the Nav2 BT nodes using the generated adapter classes ---
add_library(${PROJECT_NAME}_nav2_bt_registration SHARED 
  "${_nav2_generated_cpp}"
)
target_link_libraries(${PROJECT_NAME}_nav2_bt_registration PRIVATE
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
  pluginlib::pluginlib
)
foreach(_class_name ${_nav2_class_names})
  auto_apms_behavior_tree_register_nodes(${PROJECT_NAME}_nav2_bt_registration
    ${_class_name}
    NODE_REGISTRATION_TYPE "${_class_name}"
  )
endforeach()

# Associate the registered nodes with the generated manifest
auto_apms_behavior_tree_register_nodes(behavior_tree_nodes
  NODE_MANIFEST "${_nav2_generated_manifest}"
  NODE_MODEL_HEADER_TARGET ${PROJECT_NAME}
)

# =============================================================================
# Registration of Nav2 trees as AutoAPMS behaviors
# =============================================================================

# Register each BT XML installed by nav2_bt_navigator
file(GLOB _nav2_bt_xml_file_paths CONFIGURE_DEPENDS "${nav2_bt_navigator_DIR}/../behavior_trees/*.xml")
auto_apms_behavior_tree_register_trees(
  ${_nav2_bt_xml_file_paths}
  NODE_MANIFEST
  ${PROJECT_NAME}::behavior_tree_nodes
)

# =============================================================================
# Build handlers for initializing the existing behaviors
# =============================================================================
add_library(${PROJECT_NAME}_build_handler SHARED 
    "src/build_handler/navigate_to_pose.cpp"
)
target_link_libraries(${PROJECT_NAME}_build_handler PUBLIC
  ${PROJECT_NAME}
  auto_apms_behavior_tree::auto_apms_behavior_tree
)
# auto_apms_behavior_tree_register_build_handlers(${PROJECT_NAME}_build_handler 
#   "auto_apms_nav2::NavigateToPose"
# )

# Install shared libraries
install(
  TARGETS 
  ${PROJECT_NAME}_navigator 
  ${PROJECT_NAME}_nav2_bt_registration
  ${PROJECT_NAME}_build_handler
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install export library and export targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
