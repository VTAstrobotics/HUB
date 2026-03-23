#!/bin/bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "build.sh: [ERROR]: This script must be sourced, not executed."
    echo "build.sh: [INFO]: Usage: source build_scripts/build.sh"
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ANUBIS_DIR="$( dirname "$SCRIPT_DIR" )"   # parent of build

colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Debug

source "$ANUBIS_DIR/install/setup.sh"

#to actually use debugging look at the docs here:
# https://control.ros.org/master/doc/ros2_control/doc/debugging.html
# or alternatively use gdb with the following command:
# ros2 run --prefix 'gdb -ex run --args' <package_name> <executable_name>
#also helpful https://juraph.com/miscellaneous/ros2_and_gdb/