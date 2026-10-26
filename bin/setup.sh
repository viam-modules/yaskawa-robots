#!/bin/bash
# One-time conan setup: create a default profile and add the viamconan remote,
# which hosts prebuilt binaries for the Viam C++ SDK and its heavy dependencies.
#
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
set -euxo pipefail

conan profile detect || echo "Conan default profile already exists"

# --index 0 makes viamconan the first remote searched, ahead of conancenter.
conan remote add viamconan https://viam.jfrog.io/artifactory/api/conan/viamconan --index 0 --force
