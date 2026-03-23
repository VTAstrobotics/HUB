#!/bin/bash


if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "build.sh: [ERROR]: This script must be sourced, not executed."
    echo "build.sh: [INFO]: Usage: source build_scripts/build.sh"
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ANUBIS_DIR="$( dirname "$SCRIPT_DIR" )"   # parent of build

colcon build --parallel-workers 4 --symlink-install 

source "$ANUBIS_DIR/install/setup.sh"

