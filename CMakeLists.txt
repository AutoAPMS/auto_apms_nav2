cmake_minimum_required(VERSION 3.8)
project(auto_apms_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)

add_library(auto_apms_navigator SHARED
  src/generic_behavior_navigator.cpp
)
target_link_libraries(auto_apms_navigator PRIVATE
  pluginlib::pluginlib
  rclcpp::rclcpp
  auto_apms_behavior_tree::auto_apms_behavior_tree
)
if(TARGET nav2_core::nav2_core)
  target_link_libraries(auto_apms_navigator PRIVATE
    nav2_core::nav2_core
  )
else()
  target_include_directories(auto_apms_navigator PRIVATE
    ${nav2_core_INCLUDE_DIRS}
  )
  target_link_libraries(auto_apms_navigator PRIVATE
    ${nav2_core_LIBRARIES}
  )
endif()

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS auto_apms_navigator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
