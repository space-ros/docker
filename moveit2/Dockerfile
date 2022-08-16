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
#   VCS_REF     - The git revision of the Space ROS source code (no default value).
#   VERSION     - The version of Space ROS (default: "preview")

FROM openrobotics/spaceros:latest

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

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

# Make sure the latest versions of packages are installed
RUN sudo apt-get update
RUN sudo apt-get dist-upgrade -y
RUN rosdep update

# Define key locations
ENV MOVEIT2_DIR=${SRC_DIR}/moveit2

# Install the various build and test tools
RUN sudo apt install -y \
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

# Get the MoveIt2 source code
WORKDIR ${SRC_DIR}
RUN sudo git clone https://github.com/ros-planning/moveit2.git -b main moveit2/src 

# Get the repositories required by MoveIt2, but not included in Space ROS
WORKDIR ${MOVEIT2_DIR}
COPY --chown=${USERNAME}:${USERNAME} moveit2.repos ./
RUN sudo vcs import src < moveit2.repos

# Update the ownership of the source files (had to use sudo above to work around
# a possible inherited 'insteadof' from the host that forces use of ssh
RUN sudo chown -R ${USERNAME}:${USERNAME} ${MOVEIT2_DIR}

# Install system dependencies
RUN rosdep install --from-paths ../spaceros/src src --ignore-src --rosdistro rolling -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers rmw_connextdds ros_testing rmw_connextdds rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp composition demo_nodes_py lifecycle rosidl_typesupport_fastrtps_cpp rosidl_typesupport_fastrtps_c ikos diagnostic_aggregator diagnostic_updater joy qt_gui rqt_gui rqt_gui_py"

# Apply a patch to octomap_msgs to work around a build issue
COPY --chown=${USERNAME}:${USERNAME} octomap_fix.diff ./src/octomap_msgs
RUN cd src/octomap_msgs && git apply octomap_fix.diff

# Build MoveIt2
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Enable the container to run GUI apps and add a couple sample apps for testing
RUN touch ${HOME}/.Xauthority
EXPOSE 8887
RUN sudo apt-get install -y \
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
