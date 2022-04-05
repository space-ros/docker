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
# A Docker configuration script to build the Space ROS/MoveIt2 image.
#
# The script provides the following build arguments:
#
#   VCS_REF     - The git revision of the MoveIt2 source code (no default value)
#   VERSION     - The version of this Space ROS/MoveIt2 build (default: "preview")
#
# See the documentation for details on running the container:
#
#   https://github.com/space-ros/docker-images/tree/main/moveit2
#

FROM openrobotics/spaceros:latest

ARG VCS_REF
ARG VERSION="preview"

ARG SRCDIR=/root/src
ARG SPACEROS_WS=${SRCDIR}/spaceros_ws
ARG OCTOMAP_WS=${SRCDIR}/octomap_ws
ARG UNDERLAY_WS=${SRCDIR}/moveit2_underlay_ws
ARG MOVEIT2_WS=${SRCDIR}/moveit2_ws

# LABEL the image
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Space ROS"
LABEL org.label-schema.description="Preview version of the Space ROS/MoveIt2 platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker-images"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Make sure the latest versions of packages are installed
RUN apt-get update
RUN apt-get dist-upgrade -y

# Install the various build and test tools
RUN apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget \
  clang-format-10

# Install some pip packages needed for testing
RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

# Get the MoveIt2 underlay source code and install any required system dependencies
RUN mkdir -p ${UNDERLAY_WS}/src
WORKDIR ${UNDERLAY_WS}
COPY moveit2.repos ./
RUN vcs import src < moveit2.repos
RUN /bin/bash -c 'source ${SPACEROS_WS}/install/setup.bash && rosdep install --from-paths src --ignore-src --rosdistro rolling -y --skip-keys "urdfdom_headers"'

# Build octomap separately since it has a special 'octomap-distribution' package that needs to be built
# and sourced before building the other underlay packages
RUN /bin/bash -c 'mkdir -p ${OCTOMAP_WS}/src && mv src/octomap* ${OCTOMAP_WS}/src'
WORKDIR ${OCTOMAP_WS}
RUN /bin/bash -c 'source ${SPACEROS_WS}/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'

# Fix a compile-time issue with a misplaced header file
RUN /bin/bash -c 'cp ${OCTOMAP_WS}/install/octomap_msgs/include/octomap_msgs/conversions.h ${OCTOMAP_WS}/install/octomap_msgs/include/octomap_msgs/octomap_msgs'

# Build the underlay
WORKDIR ${UNDERLAY_WS}
RUN /bin/bash -c 'source ${OCTOMAP_WS}/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'

# Get the MoveIt2 source code and install any required system dependencies
RUN mkdir -p ${MOVEIT2_WS}/src
WORKDIR ${MOVEIT2_WS}
RUN git clone https://github.com/ros-planning/moveit2.git -b main src
RUN /bin/bash -c 'source ${UNDERLAY_WS}/install/setup.bash && rosdep install --from-paths src --ignore-src --rosdistro rolling -y --skip-keys "ruckig orocos_kdl joy octomap ompl urdfdom urdfdom_headers"'

# Build MoveIt2
RUN /bin/bash -c 'source ${UNDERLAY_WS}/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'

# Enable the container to run GUI apps and add a couple sample apps for testing
RUN touch /root/.Xauthority
EXPOSE 8887
RUN apt-get install -y \
  firefox \
  glmark2 \
  libcanberra-gtk3-0 \
  libpci-dev \
  xauth \
  xterm

# Set up the entrypoint
COPY ./moveit2_entrypoint.sh /
ENTRYPOINT ["/moveit2_entrypoint.sh"]
CMD ["bash"]
