#!/bin/bash
mkdir -p src
vcs-import src < ros2.repos
touch src/ros2/rcl_logging/rcl_logging_spdlog/COLCON_IGNORE
