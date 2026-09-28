cmake_minimum_required(VERSION 3.16)
project(wire_protocol VERSION 0.1.0 LANGUAGES CXX)

add_library(wire_protocol INTERFACE)
add_library(wire_protocol::wire_protocol ALIAS wire_protocol)
target_compile_features(wire_protocol INTERFACE cxx_std_20)
target_include_directories(wire_protocol INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS wire_protocol EXPORT wire_protocolTargets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT wire_protocolTargets NAMESPACE wire_protocol::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wire_protocol)
configure_package_config_file(cmake/wire_protocolConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/wire_protocolConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wire_protocol)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/wire_protocolConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wire_protocolConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/wire_protocolConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wire_protocol)

# 嵌入式/作为子目录使用时默认只引入头文件库。
option(WIRE_PROTOCOL_BUILD_TESTS "Build standalone protocol tests" OFF)
option(WIRE_PROTOCOL_BUILD_EXAMPLES "Build standalone examples" OFF)
if(WIRE_PROTOCOL_BUILD_TESTS)
  enable_testing()
  add_executable(wire_protocol_test tests/protocol_test.cpp)
  target_link_libraries(wire_protocol_test PRIVATE wire_protocol::wire_protocol)
  add_test(NAME wire_protocol_test COMMAND wire_protocol_test)
endif()
if(WIRE_PROTOCOL_BUILD_EXAMPLES)
  add_executable(wire_protocol_example examples/basic.cpp)
  target_link_libraries(wire_protocol_example PRIVATE wire_protocol::wire_protocol)
endif()
