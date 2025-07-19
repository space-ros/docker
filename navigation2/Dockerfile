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
# A Docker configuration script to build the Navigation2/Space ROS image.
#
# The script provides the following build arguments:
#
#   VCS_REF         - The git revision of the Space ROS source code (no default value).
#   VERSION         - The version of Space ROS (default: "preview")
#   SPACE_ROS_IMAGE - The base Space ROS image to build on

ARG SPACE_ROS_IMAGE=osrf/space-ros:jazzy-2025.04.0

FROM ${SPACE_ROS_IMAGE}

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Navigation2"
LABEL org.label-schema.description="Preview version of the Navigation2/Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker-images"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Define workspace locations
ENV NAVIGATION2_WS=${HOME_DIR}/nav2_ws

RUN mkdir -p ${NAVIGATION2_WS}/src

WORKDIR ${NAVIGATION2_WS}/

COPY navigation2.repos .
COPY excluded-pkgs.txt .
RUN vcs import --shallow src < navigation2.repos
COPY --chown=spaceros-user:spaceros-user src/ src

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \ 
    source ${SPACEROS_DIR}/install/setup.bash && \
    sudo apt update && \
    rosdep install -i --from-path src --skip-keys $(cat excluded-pkgs.txt) -y
RUN source ${SPACEROS_DIR}/install/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --no-warn-unused-cli
RUN rm -rf src build log


# Install rviz2 to send goals to Nav2. 
# TODO(asimonov): Remove rviz2 apt install in the future.
#                 It brings a lot of dependencies with it. 
#                 It installs 146 packages into /opt/ros/jazzy which is confusing as space-ros is in /opt/spaceros.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \  
    sudo apt update && \
    sudo -E apt install -y ros-${ROS_DISTRO}-rviz2

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
