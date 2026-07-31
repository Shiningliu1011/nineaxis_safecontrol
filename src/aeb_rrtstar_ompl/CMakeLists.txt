cmake_minimum_required(VERSION 3.22)
project(aeb_rrtstar_ompl)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find build dependencies (from package.xml)
find_package(ament_cmake REQUIRED)
find_package(ompl REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)

# ---- AEB-RRT* planner library ----
add_library(aeb_rrtstar_ompl_lib SHARED
  src/AEBRRTstar.cpp
)

target_include_directories(aeb_rrtstar_ompl_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(aeb_rrtstar_ompl_lib PUBLIC
  ompl::ompl
  ${Boost_LIBRARIES}
)

# ---- AEB-RRT* PlannerManager plugin ----
add_library(aeb_rrtstar_ompl_plugin SHARED
  src/aeb_rrtstar_planner_manager.cpp
)

target_include_directories(aeb_rrtstar_ompl_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(aeb_rrtstar_ompl_plugin PUBLIC
  aeb_rrtstar_ompl_lib
)

ament_target_dependencies(aeb_rrtstar_ompl_plugin PUBLIC
  moveit_planners_ompl
  moveit_ros_planning
  pluginlib
  rclcpp
)

# ---- Plugin description file ----
pluginlib_export_plugin_description_file(
  moveit_core aeb_rrtstar_plugin_description.xml)

# ---- Install ----
install(TARGETS aeb_rrtstar_ompl_lib aeb_rrtstar_ompl_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# ---- Test executables ----
add_executable(test_aeb_rrtstar src/test_aeb_rrtstar.cpp)
target_link_libraries(test_aeb_rrtstar aeb_rrtstar_ompl_lib)
target_include_directories(test_aeb_rrtstar PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OMPL_INCLUDE_DIRS}
)

add_executable(test_aeb_full src/test_aeb_full.cpp)
target_link_libraries(test_aeb_full aeb_rrtstar_ompl_lib)
target_include_directories(test_aeb_full PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OMPL_INCLUDE_DIRS}
)

install(TARGETS test_aeb_rrtstar test_aeb_full
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ---- Ament ----
ament_package()
