cmake_minimum_required(VERSION 3.8)
project(serial_transport)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wire_protocol REQUIRED)
find_path(ASIO_INCLUDE_DIR NAMES asio.hpp REQUIRED)


add_executable(serial_driver src/serial_driver.cpp)
target_include_directories(serial_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${ASIO_INCLUDE_DIR})
target_compile_features(serial_driver PUBLIC c_std_17 cxx_std_23)  # Require C17 and C++23
target_compile_definitions(serial_driver PRIVATE ASIO_STANDALONE)
ament_target_dependencies(
  serial_driver
  "rclcpp"
  "wire_protocol"

)

install(TARGETS serial_driver
  DESTINATION lib/${PROJECT_NAME})

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
