cmake_minimum_required(VERSION 3.8)
project(rs_follow_core VERSION 1.0.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# A plain CMake project on purpose.
#
# The whole point of this package is that it does NOT need ROS. It builds with:
#     cmake -S . -B build && cmake --build build
# on any machine with a C++17 compiler, and it is consumable from a ROS 2
# package (rs_follow) via find_package(). Using ament_cmake here would have
# reintroduced exactly the dependency this package exists to remove. colcon
# handles plain CMake packages through the `cmake` build type in package.xml.
# ---------------------------------------------------------------------------

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Header-only: the algorithm is a fixed-size control pipeline, and keeping it
# header-only makes the ROS wrapper's build trivial and inlining possible.
add_library(rs_follow_core INTERFACE)
target_include_directories(rs_follow_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
add_library(rs_follow_core::rs_follow_core ALIAS rs_follow_core)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS rs_follow_core EXPORT rs_follow_coreTargets)
install(EXPORT rs_follow_coreTargets
  NAMESPACE rs_follow_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rs_follow_core)

# find_package(rs_follow_core) support, so the ROS wrapper (and any other
# consumer) can link against this without hard-coding a path.
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rs_follow_coreConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/rs_follow_coreConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rs_follow_core
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/rs_follow_coreConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/rs_follow_coreConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/rs_follow_coreConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rs_follow_core)

# ---------------------------------------------------------------------------
# Standalone test. This is the proof that the decoupling works: it compiles and
# runs the real algorithm with no ROS on the include path and no ROS runtime.
#     cmake -S . -B build -DRS_FOLLOW_CORE_BUILD_TESTS=ON && ./build/test_core
# ---------------------------------------------------------------------------
option(RS_FOLLOW_CORE_BUILD_TESTS "Build the standalone (no-ROS) core tests" OFF)

if(RS_FOLLOW_CORE_BUILD_TESTS OR BUILD_TESTING)
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core PRIVATE rs_follow_core)
  if(RS_FOLLOW_CORE_BUILD_TESTS)
    enable_testing()
    add_test(NAME test_core COMMAND test_core)
  endif()
endif()
