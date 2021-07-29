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
#   BUILD_DATE  - The time and date that the docker image was created (no default value).
#   VCS_REF     - The git revision of the Space ROS source code (no default value).
#   VERSION     - The version of Space ROS (default: "preview")
#   WORKSPACE   - The location for the Space ROS source code in the container (default: /usr/local/src/spaceros_ws)

FROM ubuntu:20.04

ARG BUILD_DATE
ARG VCS_REF
ARG VERSION="preview"
ARG WORKSPACE=/usr/local/src/spaceros_ws

# LABEL the image
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Space ROS"
LABEL org.label-schema.description="Preview version of the Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://openrobotics.org"
LABEL org.label-schema.vcs-url="https://github.com/ros2/ros2/tree/spaceros"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# The following commands are based on the source install for ROS 2 Rolling Ridley.
# See: https://docs.ros.org/en/ros2_documentation/rolling/Installation/Ubuntu-Development-Setup.html
# The only variation is getting Space ROS sources instead of the Rolling sources.

# Update the Ubuntu software repository
RUN apt-get update

# Set the locale
RUN apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add the ROS 2 apt repository
RUN apt-get install -y software-properties-common
RUN add-apt-repository universe
RUN apt-get update && apt-get install -y curl gnupg lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install required software development tools and ROS tools
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

RUN python3 -m pip install -U \
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
  pytest \
  setuptools

# Get Space ROS source code
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}
RUN wget https://raw.githubusercontent.com/ros2/ros2/spaceros/ros2.repos
RUN vcs import src < ros2.repos

# Install dependencies using rosdep
RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro rolling -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

# Build the code in the workspace
RUN colcon build --symlink-install

# Set up the entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
