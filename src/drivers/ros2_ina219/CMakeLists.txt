cmake_minimum_required(VERSION 3.8)
project(ina219_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(ina219_node src/ina219_node.cpp)
target_include_directories(ina219_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ina219_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(ina219_node rclcpp std_msgs)

install(TARGETS 
ina219_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()