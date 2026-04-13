cmake_policy(SET CMP0074 NEW)
cmake_minimum_required(VERSION 3.8)
project(amr_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(Python_EXECUTABLE /usr/bin/python3)
set(Python_INCLUDE_DIR /usr/include/python3.10)
set(Python_LIBRARY /usr/lib/x86_64-linux-gnu/libpython3.10.so)

find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development)



# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT
)

list(APPEND CMAKE_PREFIX_PATH ${nanobind_ROOT})

find_package(nanobind CONFIG REQUIRED)

# add here the python modules every time we make a new function
nanobind_add_module(cpp_module
  src/bindings.cpp
  src/sense.cpp
  src/resample.cpp
)

install(TARGETS cpp_module
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

ament_package()
