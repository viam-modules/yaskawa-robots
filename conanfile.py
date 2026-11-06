import os

from conan import ConanFile
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, load
from conan.tools.scm import Git

SDK_VERSION_FILE = "etc/viam-cpp-sdk.version"

class yaskawa_robots(ConanFile):
    name = "viam-yaskawa-robots"

    # requirements() reads the SDK version from the recipe folder, so the file
    # must be exported alongside the recipe to exist in the conan cache.
    exports = SDK_VERSION_FILE

    license = "Apache-2.0"
    url = "https://github.com/viam-modules/yaskawa-robots"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"

    options = {
        "shared": [True, False]
    }

    default_options = {
        "shared": True
    }

    def export_sources(self):
        for pat in ["CMakeLists.txt", "LICENSE", "src/*", "meta.json*", "*.sh", "README.md", SDK_VERSION_FILE]:
            copy(self, pat, self.recipe_folder, self.export_sources_folder)

    def set_version(self):
        git = Git(self)
        try:
            self.version = git.run("describe --tags --always").strip().lstrip('v')
        except Exception:
            self.version = "0.0.0-unknown"

    def requirements(self):
        # Prebuilt binaries on viamconan only match the exact dependency versions
        # the SDK release was built with, which conan.lock pins. If you bump the
        # SDK in etc/viam-cpp-sdk.version, regenerate conan.lock from that
        # release's lockfile (see README).
        sdk_version = load(self, os.path.join(self.recipe_folder, SDK_VERSION_FILE)).strip()
        self.requires(f"viam-cpp-sdk/{sdk_version}")
        self.requires("eigen/[>=3.4 <5.0]")
        self.requires("boost/[>=1.80.0]")
        self.requires("jsoncpp/[>=1.9.5]")
        self.requires("grpc/[>=1.48.4]")

    def validate(self):
        check_min_cppstd(self, 20)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()
        CMakeDeps(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        # Build the default `all` target: the module plus the installable trajex
        # targets that package() installs. The yaskawa test executables are all
        # EXCLUDE_FROM_ALL, so they (and the third-party comms C they pull in) are
        # not compiled here; they run in the Build/Test/Lint jobs via `make test`.
        cmake.build()

    def layout(self):
        cmake_layout(self, src_folder=".")

    def package(self):
        CMake(self).install()

        # Use CPack to build the module.tar.gz and manually copy it to the package folder
        CMake(self).build(target='package')
        copy(self, pattern="module.tar.gz", src=self.build_folder, dst=self.package_folder)

    def deploy(self):
        copy(self, pattern="module.tar.gz", src=self.package_folder, dst=self.deploy_folder)
