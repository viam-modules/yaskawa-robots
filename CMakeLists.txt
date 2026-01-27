cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(
  yaskawa-robots
  DESCRIPTION "Viam Module to support Yaskawa Robots arms"
  HOMEPAGE_URL https://github.com/viam-modules/yaskawa-robots
  LANGUAGES CXX
)

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
  GIT_TAG releases/v0.30.0 # Keep in sync with Dockerfile
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
find_package(viam-cpp-sdk 0.30 CONFIG REQUIRED viamsdk)

# Everything needs threads, and prefer -pthread if available
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Get boost pulled in
set(BOOST_MIN_VERSION "1.80.0")
find_package(Boost CONFIG REQUIRED
  COMPONENTS
  headers
  regex
  system
  thread
  unit_test_framework
)

enable_testing()

#
# Declare targets
#

add_library(trajectories)

set_target_properties(trajectories PROPERTIES SYSTEM TRUE)

target_sources(trajectories
  PRIVATE
    src/third_party/trajectories/Trajectory.cpp
    src/third_party/trajectories/Path.cpp
)

target_include_directories(trajectories
  INTERFACE
    src
)

target_link_libraries(trajectories
  PUBLIC
    Eigen3::Eigen
)

install(
  TARGETS
    trajectories
)


add_library(viam-yaskawa)

#
target_sources(viam-yaskawa
  PRIVATE
    src/viam/lib/robot_socket.cpp
    src/viam/lib/logger.cpp
)

target_compile_options( viam-yaskawa PRIVATE -O0)

# Tell boost::dll to use std::filesystem rather than boost::filesystem.
target_compile_definitions(viam-yaskawa
  PRIVATE
    BOOST_DLL_USE_STD_FS
)

target_link_libraries(viam-yaskawa
  PUBLIC
    trajectories
    Boost::regex
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


add_executable(yaskawa-example)

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
  DIRECTORY src/kinematics
  DESTINATION ${CMAKE_INSTALL_DATADIR}/yaskawa-robots
)

add_executable(broadcast_log_parser_test)

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

set(CPACK_PACKAGE_NAME "viam-yaskawa-robots")
set(CPACK_PACKAGE_FILE_NAME "module")
set(CPACK_GENERATOR "TGZ")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
include(CPack)
