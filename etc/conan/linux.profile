# Toolchain the Viam C++ SDK publishes its Linux binaries with: the default
# profile of the ghcr.io/viamrobotics/cpp-sdk-conan-ubuntu:noble image.
# bin/build.sh passes it with -pr:a so our dependency package IDs match those
# binaries. A `conan profile detect` profile often differs (e.g. cppstd=gnu17),
# which silently misses the cache and rebuilds everything from source.
#
# compiler.cppstd is 17 to match the published dependencies; bin/build.sh
# raises only our own package to C++20. gcc 13 is required for std::format.
# Keep in sync when bumping the SDK.
include(default)

[settings]
os=Linux
compiler=gcc
compiler.version=13
compiler.libcxx=libstdc++11
compiler.cppstd=17
build_type=Release
