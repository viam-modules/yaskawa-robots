cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(
  yaskawa-robots
  DESCRIPTION "Viam Module to support Yaskawa Robots arms"
  HOMEPAGE_URL https://github.com/viam-modules/yaskawa-robots
  LANGUAGES CXX C
)

if(APPLE AND NOT CMAKE_OSX_DEPLOYMENT_TARGET)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "14.0")
endif()

# - `VIAM_YASKAWA_USE_WALL_WERROR`
#
# This causes the internal code to compile with `-Wall` and
# `-Werror` flags.
#
option(VIAM_YASKAWA_USE_WALL_WERROR "Build with -Wall and -Werror flags" ON)


# If no build type is selected, build optimized but retain debug
# info. This is probably the right build type for packaging and
# release.
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()


# Enforce the C++ standard, and disable extensions
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)


if (MSVC)
  # https://discourse.cmake.org/t/set-cmake-cxx-standard-should-set-zc-cplusplus-for-msvc/1876
  string(APPEND CMAKE_CXX_FLAGS " /Zc:__cplusplus")
endif()
set(CMAKE_CXX_EXTENSIONS OFF)


# Produce a compilation database when generating for a build
# system that is able to make one.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Produce GNU-style variables for installation, and default the
# installation directory to be local to the build. If you intend to
# install elsewhere, pass an explicit argument to CMAKE_INSTALL_PREFIX
# on the command line:
#
# cmake ... -DCMAKE_INSTALL_PREFIX=$HOME/opt
#
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "..." FORCE)
endif()
include(GNUInstallDirs)

# If we aren't doing an app image build, configure runtime paths for relocatable installs
set(CMAKE_BUILD_WITH_INSTALL_RPATH false)
set(CMAKE_BUILD_RPATH_USE_ORIGIN true)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH true)
set(CMAKE_SKIP_BUILD_RPATH false)

list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  # TODO: Is there a way to do this with generator expressions?
  file(RELATIVE_PATH BINTOPREFIXRELPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR} ${CMAKE_INSTALL_PREFIX})
  file(RELATIVE_PATH PREFIXTOLIBRELPATH ${CMAKE_INSTALL_PREFIX} ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

  if (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    list(PREPEND CMAKE_INSTALL_RPATH "@loader_path/${BINTOPREFIXRELPATH}/${PREFIXTOLIBRELPATH}")
  else()
    list(PREPEND CMAKE_INSTALL_RPATH "$ORIGIN/${BINTOPREFIXRELPATH}/${PREFIXTOLIBRELPATH}")
  endif()
endif()


#
# Obtain dependencies via FetchContent / find_package
#

include(FetchContent)

FetchContent_Declare(
  viam-cpp-sdk
  GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk
  GIT_TAG releases/v0.31.0 # Keep in sync with Dockerfile
  GIT_SHALLOW TRUE
  SYSTEM
  # Without this patch command, using this as a SYSTEM package results
  # in failed compilation due to the statically included protobuf gens
  # getting in the wrong order. Just delete them, we don't need them.
  PATCH_COMMAND find ./src/viam/api -name "*.pb.h" -type f -exec rm {} +
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(viam-cpp-sdk)

find_package(Eigen3 3.4 CONFIG REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(viam-cpp-sdk 0.31 CONFIG REQUIRED viamsdk)

# Everything needs threads, and prefer -pthread if available
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Get boost pulled in
set(BOOST_MIN_VERSION "1.80.0")

# Boost::system became header-only in 1.85.0 and no longer needs explicit linking.
# Make it optional to support both old and new versions.
find_package(Boost ${BOOST_MIN_VERSION} CONFIG REQUIRED
  COMPONENTS
    headers
    regex
    thread
    unit_test_framework
  OPTIONAL_COMPONENTS
    system
)

enable_testing()

#
# Declare targets
#

FetchContent_Declare(
  trajex
  GIT_REPOSITORY https://github.com/viam-modules/trajex.git
  GIT_TAG v0.100.0-rc1
)

find_package(xtl CONFIG REQUIRED)
find_package(xtensor CONFIG REQUIRED)

FetchContent_MakeAvailable(trajex)

FetchContent_Declare(
  comms
  GIT_REPOSITORY https://github.com/viam-labs/viam-yaskawa-libs.git
    GIT_TAG v0.2.0
  GIT_SHALLOW TRUE
  SYSTEM
)

# Build the comms library (needed for integration tests) but skip its own tests/examples
set(COMMS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(COMMS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(COMMS_PLATFORM_POSIX ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(comms)

# Suppress conversion warnings from third-party C code that we don't control.
if(TARGET comms)
  target_compile_options(comms PRIVATE
    -Wno-conversion
    $<$<BOOL:${APPLE}>:-Wno-deprecated-declarations>
  )
endif()

add_library(viam-yaskawa)

#
target_sources(viam-yaskawa
  PRIVATE
    src/viam/lib/robot_socket.cpp
    src/viam/lib/logger.cpp
    src/viam/lib/trajectory_logger.cpp
)

target_include_directories(viam-yaskawa
  PUBLIC
    src
    ${comms_SOURCE_DIR}
)

target_compile_options( viam-yaskawa PRIVATE -O0)

# Tell boost::dll to use std::filesystem rather than boost::filesystem.
target_compile_definitions(viam-yaskawa
  PRIVATE
    BOOST_DLL_USE_STD_FS
  PUBLIC
    $<$<CXX_COMPILER_ID:Clang,AppleClang>:_Bool=bool>
)

target_link_libraries(viam-yaskawa
  PUBLIC
    viam::trajex::totg::tools
    Boost::regex
    Eigen3::Eigen
    jsoncpp_lib
    viam-cpp-sdk::viamsdk
)

install(
  TARGETS
    viam-yaskawa
)

# If we have opted into more extensive warnings as errors, apply them now
if (VIAM_YASKAWA_USE_WALL_WERROR)
  set(GNULIKE_COMPILERS "Clang" "AppleClang" "GNU")
  if (CMAKE_CXX_COMPILER_ID IN_LIST GNULIKE_COMPILERS)
    # TODO: Arguably, these should be `PRIVATE`.
    target_compile_options(viam-yaskawa PUBLIC
      -Wall
      -Werror
      -Wextra
      -pedantic
      -Wconversion
      -Wdouble-promotion
      -Wenum-conversion
      -Wnull-dereference
      -Wunused
    )
  elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(viam-yaskawa PRIVATE /W4 /WX)
  else()
    message(FATAL_ERROR "VIAM_YASKAWA_USE_WALL_ERROR is set, but not known how to enable for compiler ID ${CMAKE_CXX_COMPILER_ID}")
  endif()
endif()


add_executable(yaskawa-robots)

#Compute the relative path from the bindir to the data
# directory. This is relocatable such that if we later install to a
# different prefix it will still be correct, but we need to use the
# full paths to compute it.
file(RELATIVE_PATH VIAM_YASKAWA_RELPATH_BINDIR_TO_DATADIR ${CMAKE_INSTALL_FULL_BINDIR} ${CMAKE_INSTALL_FULL_DATADIR})

configure_file(
  src/viam/module/yaskawa_arm_config.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/src/viam/module/yaskawa_arm_config.hpp
  @ONLY
)

target_include_directories(yaskawa-robots
  PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/src/viam/module/
)


target_sources(yaskawa-robots
  PRIVATE
  src/viam/module/main.cpp
  src/viam/module/utils.cpp
  src/viam/module/yaskawa_arm.cpp
  src/viam/module/yaskawa_arm_state.cpp
  src/viam/module/yaskawa_arm_state_events.cpp
  src/viam/module/yaskawa_arm_state_disconnected.cpp
  src/viam/module/yaskawa_arm_state_independent.cpp
  src/viam/module/yaskawa_arm_state_ready.cpp
)

# Tell boost::dll to use std::filesystem rather than boost::filesystem.
target_compile_definitions(yaskawa-robots
  PRIVATE
    BOOST_DLL_USE_STD_FS
)

target_link_libraries(yaskawa-robots
  PRIVATE
  viam-yaskawa
  viam-cpp-sdk::viamsdk
)
target_compile_options( yaskawa-robots PRIVATE -O0)

install(
  TARGETS
    yaskawa-robots
)


add_executable(yaskawa-example EXCLUDE_FROM_ALL)

target_sources(yaskawa-example
  PRIVATE
    src/viam/example/main.cpp
)

target_link_libraries(yaskawa-example
  PRIVATE
  viam-yaskawa
  viam-cpp-sdk::viamsdk
)

target_include_directories(yaskawa-example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/viam/lib)

target_compile_options( yaskawa-example PRIVATE -O0)


#
# Installation and packaging
#

configure_file(
    meta.json.in
    ${CMAKE_CURRENT_BINARY_DIR}/meta.json
    @ONLY
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/meta.json
  DESTINATION .
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/README.md
  DESTINATION .
)

install(
  DIRECTORY src/kinematics
  DESTINATION ${CMAKE_INSTALL_DATADIR}/yaskawa-robots
)

add_executable(broadcast_log_parser_test EXCLUDE_FROM_ALL)

target_sources(broadcast_log_parser_test
  PRIVATE
    src/viam/lib/test/broadcast_log_parser_test.cpp
)

target_link_libraries(broadcast_log_parser_test
  PRIVATE
    viam-yaskawa
    Boost::unit_test_framework
)

add_test(NAME broadcast_log_parser_test COMMAND broadcast_log_parser_test)

add_executable(realtime_trajectory_logger_test EXCLUDE_FROM_ALL)

target_sources(realtime_trajectory_logger_test
  PRIVATE
    src/viam/lib/test/realtime_trajectory_logger_test.cpp
)

target_link_libraries(realtime_trajectory_logger_test
  PRIVATE
    viam-yaskawa
    Boost::unit_test_framework
)

add_test(NAME realtime_trajectory_logger_test COMMAND realtime_trajectory_logger_test)

add_executable(move_limit_test EXCLUDE_FROM_ALL)

target_sources(move_limit_test
  PRIVATE
    src/viam/module/test/move_limit_test.cpp
    src/viam/module/utils.cpp
)

target_link_libraries(move_limit_test
  PRIVATE
    viam-yaskawa
    Boost::unit_test_framework
    viam-cpp-sdk::viamsdk
)

add_test(NAME move_limit_test COMMAND move_limit_test)

# Common sources and include dirs for integration tests that use the fake server
set(INTEGRATION_TEST_SOURCES
  src/viam/lib/test/fake_server.cpp
  ${comms_SOURCE_DIR}/tests/protocol/mock_robot.c
)
set(INTEGRATION_TEST_INCLUDE_DIRS
  ${comms_SOURCE_DIR}
  ${comms_SOURCE_DIR}/tests/protocol
)


add_executable(fault_injection_test EXCLUDE_FROM_ALL)

target_sources(fault_injection_test
  PRIVATE
    src/viam/lib/test/fault_injection_test.cpp
    ${INTEGRATION_TEST_SOURCES}
    ${comms_SOURCE_DIR}/tests/protocol/fault_inject.c
)

target_include_directories(fault_injection_test
  PRIVATE
    ${INTEGRATION_TEST_INCLUDE_DIRS}
)

target_link_libraries(fault_injection_test
  PRIVATE
    viam-yaskawa
    comms
    Boost::unit_test_framework
    viam-cpp-sdk::viamsdk
)


# Suppress warnings on third-party C sources compiled into this target.
set_source_files_properties(
  ${INTEGRATION_TEST_SOURCES}
  ${comms_SOURCE_DIR}/tests/protocol/fault_inject.c
  PROPERTIES COMPILE_OPTIONS "-Wno-conversion"
)

add_test(NAME fault_injection_test COMMAND fault_injection_test)

set(CPACK_PACKAGE_NAME "viam-yaskawa-robots")
set(CPACK_PACKAGE_FILE_NAME "module")
set(CPACK_GENERATOR "TGZ")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
include(CPack)
