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

FROM openrobotics/spaceros:latest

ARG VCS_REF
ARG VERSION="preview"
ARG WORKSPACE=/root/src/spaceros_demo_ws

WORKDIR ${WORKSPACE}

# LABEL the image
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Space ROS"
LABEL org.label-schema.description="Preview demo of the Space ROS platform"
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
RUN DEBIAN_FRONTEND=noninteractive apt install -y python3-pip wget lsb-release gnupg2 tmux g++ gdb curl

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

# Get the depends source code
RUN mkdir -p ${WORKSPACE}/src
COPY demo_spaceros.repos ./

# # Get the repositories, but not included in Space ROS
# RUN vcs import src < demo_spaceros.repos

# Build demos
RUN /bin/bash -c 'source /root/src/spaceros_ws/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'

# Get ign-gazebo6
# RUN /bin/bash -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
# RUN apt-get update
# RUN apt-get install -y libignition-gazebo6-dev


# Set up the entrypoint
WORKDIR ${WORKSPACE}
COPY ./demo_entrypoint.sh /
ENTRYPOINT ["/demo_entrypoint.sh"]
CMD ["bash"]

#rosinstall_generator --rosdistro rolling --from-path ros_gz --deps --format repos


