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
# A Docker configuration script to build the MoveIt2/Space ROS image.
#
# The script provides the following build arguments:
#
#   VCS_REF         - The git revision of the Space ROS source code (no default value).
#   VERSION         - The version of Space ROS (default: "preview")
#   SPACE_ROS_IMAGE - The base Space ROS image to build on

ARG SPACE_ROS_IMAGE=osrf/space-ros:jazzy-2025.10.0

FROM ${SPACE_ROS_IMAGE}

SHELL [ "/bin/bash", "-o", "pipefail", "-c" ]

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"
ARG USERNAME="spaceros-user"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="MoveIt2"
LABEL org.label-schema.description="Preview version of the MoveIt2/Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker-images"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Update the ROS package keys
ADD --chmod=644 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros-archive-keyring.gpg

# Define key locations
ENV MOVEIT2_DIR=${HOME}/moveit2

# Make sure the latest versions of packages are installed
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN sudo rosdep init
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get dist-upgrade -y
RUN rosdep update

# Install the various build and test tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo -E apt install -y \
    build-essential \
    clang-format \
    cmake \
    git \
    libbullet-dev \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-setuptools \
    python3-vcstool \
    wget

# Get the MoveIt2 source code
RUN git clone https://github.com/ros-planning/moveit2.git -b ${ROS_DISTRO} ${MOVEIT2_DIR}/src/moveit2

# Get a list of all installed spaceros sources
RUN source "${SPACEROS_DIR}/setup.bash" && \
    ros2 pkg list > /tmp/spaceros-pkgs.txt

# Generate repos file for moveit2 dependencies, excluding packages from Space ROS core.
COPY --chown=${USERNAME}:${USERNAME} moveit2-pkgs.txt /tmp/
COPY --chown=${USERNAME}:${USERNAME} excluded-pkgs.txt /tmp/
RUN rosinstall_generator \
    --rosdistro ${ROS_DISTRO} \
    --deps \
    --exclude $(cat /tmp/spaceros-pkgs.txt /tmp/excluded-pkgs.txt) -- \
    -- $(cat /tmp/moveit2-pkgs.txt) \
    > ${MOVEIT2_DIR}/moveit2_generated_pkgs.repos

# Get the repositories required by MoveIt2, but not included in Space ROS
WORKDIR ${MOVEIT2_DIR}
RUN vcs import --shallow src < ${MOVEIT2_DIR}/moveit2_generated_pkgs.repos

# Install system dependencies
COPY --chown=${USERNAME}:${USERNAME} rosdep-skip.txt /tmp/
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    source ${SPACEROS_DIR}/setup.bash && \
    sudo apt-get update && \
    rosdep install \
      --from-paths src \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} \
      -r -y \
      --skip-keys "$(cat /tmp/rosdep-skip.txt | tr '\n' ' ')"

# Build MoveIt2
# We temporarily disable OMPL's demos due to a build error.
RUN source ${SPACEROS_DIR}/setup.bash && \
    colcon build \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -DOMPL_BUILD_DEMOS=OFF && \
    rm -rf src build log

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
