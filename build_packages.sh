#! /bin/bash

# This is a short reference of the commands to build
# packages for this library.
SELF_DIR=$(dirname "$0")

cd "$SELF_DIR"
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
cpack ..
