cmake_minimum_required(VERSION 3.8)
project(palletrone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(palletrone_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)

# wrench_controller
add_executable(wrench_controller src/wrench_controller.cpp)
ament_target_dependencies(wrench_controller rclcpp palletrone_interfaces)
target_link_libraries(wrench_controller Eigen3::Eigen)

# allocator_controller
add_executable(allocator_controller src/allocator_controller.cpp)
ament_target_dependencies(allocator_controller rclcpp palletrone_interfaces)
target_link_libraries(allocator_controller Eigen3::Eigen)

# torque_dob
add_executable(torque_dob src/torque_dob.cpp)
ament_target_dependencies(torque_dob rclcpp palletrone_interfaces std_msgs)
target_link_libraries(torque_dob Eigen3::Eigen)

# C++ target install
install(TARGETS
  wrench_controller
  allocator_controller
  torque_dob
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
