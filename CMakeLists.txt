cmake_minimum_required(VERSION 3.5)
project(turtle_estimation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(turtle_interfaces REQUIRED)
find_package(turtle_common REQUIRED)
find_package(turtle_description REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(Eigen3 REQUIRED)

# add_library(KalmanFilter src/KalmanFilter.cpp)
# target_include_directories(KalmanFilter PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(KalmanNode Eigen3::Eigen)
# ament_target_dependencies(KalmanFilter ament_index_cpp)
add_library(ExtendedKalmanFilter src/ExtendedKalmanFilter.cpp)
target_include_directories(ExtendedKalmanFilter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ExtendedKalmanFilter Eigen3::Eigen)
ament_target_dependencies(ExtendedKalmanFilter rclcpp turtle_common)

add_library(EstimationNode src/EstimationNode.cpp)
target_include_directories(EstimationNode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(EstimationNode Eigen3::Eigen ExtendedKalmanFilter)
ament_target_dependencies(EstimationNode rclcpp std_msgs sensor_msgs nav_msgs turtle_interfaces turtle_common yaml_cpp_vendor turtle_description tf2_ros ament_index_cpp)

add_executable(state_estimation src/state_estimation.cpp)
target_include_directories(state_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(state_estimation EstimationNode)
ament_target_dependencies(state_estimation rclcpp)

install(TARGETS state_estimation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch
    DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
