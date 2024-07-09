# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# A Simple container with ROS2+Rviz+Nav2, for visualization and goal setting
# The script provides the following build arguments:
#
#   BASE_ROSDISTRO     - The distribution of ROS2 to be used as a base image

ARG BASE_ROSDISTRO
FROM osrf/ros:${BASE_ROSDISTRO}-desktop-full

WORKDIR /opt/app

#spaceros uses cyclonedds by default
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
RUN apt-get update && \
    apt-get install -y \
        ros-$ROS_DISTRO-nav2-bringup \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp
