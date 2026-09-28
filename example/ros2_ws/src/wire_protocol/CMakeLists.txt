cmake_minimum_required(VERSION 3.16)
project(wire_protocol VERSION 0.1.0 LANGUAGES CXX)

add_library(${PROJECT_NAME}
  src/protocol.cpp
)
add_library(wire_protocol::wire_protocol ALIAS ${PROJECT_NAME})
target_compile_features(serial_comm PUBLIC c_std_17 cxx_std_20)  # Require C17 and C++20
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS ${PROJECT_NAME}
  EXPORT wire_protocolTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(EXPORT wire_protocolTargets
  NAMESPACE wire_protocol::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wire_protocol
)

# 在构建目录生成包配置，无需在源码包中保留 cmake/ 目录。
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/wire_protocolConfig.cmake"
  "include(\"\${CMAKE_CURRENT_LIST_DIR}/wire_protocolTargets.cmake\")\n")
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/wire_protocolConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/wire_protocolConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/wire_protocolConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wire_protocol
)
