#!/bin/bash

IMAGE_NAME="osrf/ros2"

set -e

# Build the stack

# Helper function
function help {
    echo "Usage: $0 stack-name [options]"
    echo "USAGE"
    echo "-h, --help: display this help message"
    echo "  stack-name: the name of the stack to build. Must be one of the following:"
    echo "    - gui"
    echo "    - nav2"
    echo "    - moveit2"
}

function build_gui {
    echo "Building GUI stack"
    docker build -t $IMAGE_NAME:gui -f Dockerfile.gui .
}

# Parse command line arguments
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    help
    exit 0
fi

if [ "$1" == "gui" ]; then
    build_gui
else
    echo "Invalid stack name"
    help
    exit 1
fi
