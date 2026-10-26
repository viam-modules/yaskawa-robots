# Toolchain the Viam C++ SDK publishes its darwin binaries with; copied from the
# SDK's etc/conan/macos.profile. bin/build.sh passes it with -pr:a so our
# package IDs match those binaries. Keep in sync when bumping the SDK.
include(default)

# macOS 14 Sonoma
[settings]
os=Macos
os.version=14.0
arch=armv8
compiler=apple-clang
compiler.version=21
compiler.libcxx=libc++
compiler.cppstd=17
build_type=Release
