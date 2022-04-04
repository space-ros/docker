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
# A Docker configuration script to build the Space ROS image.
#
# The script provides the following build arguments:
#
#   VCS_REF     - The git revision of the Space ROS source code (no default value).
#   VERSION     - The version of Space ROS (default: "preview")
#   WORKSPACE   - The location for the Space ROS source code in the container (default: /root/src/spaceros_ws)
#
# Launch the container:
#   docker run -it --gpus all --net=host -e DISPLAY -v /tmp/.X11-unix openrobotics/moveit2
#
# Run xauth add inside the container (get the values from the host system via 'xauth list'):
#   xauth add <hostname>/unix:1  MIT-MAGIC-COOKIE-1  <cookie-value>
#
# Launch the MoveGroup demo in side the container:
#   ros2 launch run_move_group run_move_group.launch.py
#
# Launch another terminal into the same or a new container and run:
#   ros2 launch run_move_group run_move_group_interface.launch.py

FROM openrobotics/spaceros:latest

ARG VCS_REF
ARG VERSION="preview"
ARG WORKSPACE=/root/src/moveit2_ws

WORKDIR ${WORKSPACE}

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

# Get the MoveIt2 source code
RUN mkdir -p ${WORKSPACE}/src
COPY moveit2.repos ./
RUN git clone https://github.com/ros-planning/moveit2.git -b main src/moveit2

# Get the repositories required by MoveIt2, but not included in Space ROS
RUN vcs import src < moveit2.repos

# Install system dependencies
RUN /bin/bash -c 'source /root/src/spaceros_ws/install/setup.bash && rosdep install --from-paths ../spaceros_ws/src src --ignore-src --rosdistro rolling -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers rmw_connextdds ros_testing rmw_connextdds rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp composition demo_nodes_py lifecycle rosidl_typesupport_fastrtps_c ikos diagnostic_aggregator diagnostic_updater joy"'

# Build MoveIt2
RUN /bin/bash -c 'source /root/src/spaceros_ws/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'

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
