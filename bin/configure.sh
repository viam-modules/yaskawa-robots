#!/bin/bash
# Configure a CMake build tree in ./build against conan-provided dependencies,
# for development and the Build/Test/Lint CI job. Run ./bin/setup.sh first.
#
# Unlike bin/build.sh, this does not package the module: it only installs the
# dependencies and generates build/compile_commands.json, so the usual
# `make build`, `make test`, and `make run-clang-tidy` targets work on it.
#
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
set -euxo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ "$(uname -s)" == "Darwin" ]]; then
    PROFILE=${REPO_ROOT}/etc/conan/macos.profile
else
    PROFILE=${REPO_ROOT}/etc/conan/linux.profile
fi

CONAN_OUT=${REPO_ROOT}/build/conan

# Profile, lockfile, settings, and options must match bin/build.sh so the
# dependency package IDs match the prebuilt viamconan binaries. `&` is set to
# RelWithDebInfo so CMakeDeps generates config files for the CMake build type
# below; the dependencies themselves stay on the profile's build type.
conan install "${REPO_ROOT}" \
      --output-folder="${CONAN_OUT}" \
      --profile:all="${PROFILE}" \
      --lockfile="${REPO_ROOT}/conan.lock" \
      --build=missing \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a "&:compiler.cppstd=20" \
      -o:a "*:shared=False" \
      -o:a "&:shared=False" \
      -o:a "boost/*:without_cobalt=True"

cmake -S "${REPO_ROOT}" -B "${REPO_ROOT}/build" -G Ninja \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_TOOLCHAIN_FILE="${CONAN_OUT}/build/RelWithDebInfo/generators/conan_toolchain.cmake"
