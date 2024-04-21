cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(diff_control src/diff_control.cpp)

target_include_directories(diff_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(diff_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  diff_control
  "std_msgs"
  "rclcpp"
)

install(TARGETS diff_control
  DESTINATION lib/${PROJECT_NAME})
  
# Install launch files.
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
