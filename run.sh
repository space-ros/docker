#!/bin/bash

IMAGE_NAME="osrf/ros2"
CONTAINER_NAME="space-ros"

set -e

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

function run_gui {
    echo "Running GUI stack"
    docker run -it --rm --name $CONTAINER_NAME-gui \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -e __NV_PRIME_RENDER_OFFLOAD=1 \
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e XAUTHORITY=$XAUTHORITY \
        -v /run/user:/run/user:ro \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        $IMAGE_NAME:gui

}

# Parse command line arguments
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    help
    exit 0
fi

if [ "$1" == "gui" ]; then
    run_gui
else
    echo "Invalid stack name"
    help
    exit 1
fi
