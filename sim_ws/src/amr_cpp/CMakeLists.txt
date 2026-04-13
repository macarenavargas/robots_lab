cmake_minimum_required(VERSION 3.8)
project(amr_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nanobind REQUIRED) # i added this 

# Add C++ "include" directory
include_directories(include)

# Create C++ executables
add_executable(lifecycle src/lifecycle_node.cpp)
ament_target_dependencies(lifecycle rclcpp rclcpp_lifecycle)

# i added this add module 
nanobind_add_module(pf_cpp
  NB_STATIC       
  src/bindings.cpp
  src/pf_core.cpp
)

# Install C++ executables
install(TARGETS
  lifecycle
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pf_cpp
  LIBRARY DESTINATION lib
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
