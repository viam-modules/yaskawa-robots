#!/bin/bash
# Build the module with conan and write module.tar.gz to the repo root.
#
# Expects `conan` on PATH with a default profile and the viamconan remote
# configured; run ./bin/setup.sh once to do that.
#
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
set -euxo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

# Dependencies are downloaded prebuilt from viamconan rather than compiled,
# which only works if our settings produce the same package IDs the SDK
# release published. conan.lock pins the exact versions it was built with, and
# the profile must match the one it was built under: C++17, Release, static.
# Only our own package (`&`) and the SDK's build type deviate from that; the
# SDK publishes a RelWithDebInfo variant so its stack frames stay readable.
#
# On Linux, the default profile of the C++ SDK's Ubuntu Noble conan image
# (gcc 13, needed for std::format) is the toolchain of record. On macOS, use
# the checked-in profile the SDK publishes its darwin binaries with.
if [[ "$(uname -s)" == "Darwin" ]]; then
    PROFILE=./etc/conan/macos.profile
else
    PROFILE=default
fi

CONAN_ARGS=(
    -pr:a "${PROFILE}"
    --lockfile=conan.lock
    -s:a "&:compiler.cppstd=20"
    -s:a "&:build_type=RelWithDebInfo"
    -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo"
)

VERSION=$(conan inspect -vquiet . --format=json | python3 -c 'import json, sys; print(json.load(sys.stdin)["version"])')

conan create . "${CONAN_ARGS[@]}" --build=missing

# Copy module.tar.gz out of the conan cache into the repo root.
conan install --requires="viam-yaskawa-robots/${VERSION}" \
    "${CONAN_ARGS[@]}" \
    --lockfile-partial \
    --build=never \
    --deployer-package "&" \
    --deployer-folder=. \
    --envs-generation false
