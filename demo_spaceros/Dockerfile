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

# Install the various build and test tools
RUN apt install -y \
  build-essential \
  cmake \
  curl \
  g++ \
  gdb \
  git \
  gnupg2 \
  libbullet-dev \
  lsb-release \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget \
  clang-format-10 \
  tmux \
  unzip


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



# Get OpenCV
WORKDIR ${WORKSPACE}/..
RUN apt install -y libboost-python-dev
RUN mkdir -p opencv
WORKDIR /root/src/opencv
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
RUN unzip opencv.zip
RUN mkdir -p build
WORKDIR /root/src/opencv/build
RUN cmake  ../opencv-4.x
RUN cmake --build .
ENV OpenCV_DIR /root/src/opencv/build

# Get the depends source code
WORKDIR ${WORKSPACE}/..
RUN mkdir -p depends_ws/src

# # Get the repositories, but not included in Space ROS
WORKDIR /root/src/depends_ws
COPY demo_spaceros.repos ./
RUN vcs import src < demo_spaceros.repos



# Get dependencies of dependencies
RUN apt install -y \
    libxaw7-dev \
    libxrandr-dev \
    libasio-dev \
    qtbase5-dev \
    qtdeclarative5-dev \
    python3-pyqt5 \
    sip-dev \
    pyqt5-dev \
    python-sip-dev \
    pyqt5-dev-tools \
    libgflags-dev \
    libgtest-dev \
    libogg-dev \
    libtheora-dev \
    libfreetype6-dev

# Get ign-gazebo6
RUN /bin/bash -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y \
    libignition-math6-dev \
    libignition-transport11-dev \
    libignition-gazebo6-dev 

ENV IGNITION_VERSION fortress


# Set up the entrypoint
WORKDIR /root/src/depends_ws
# RUN /bin/bash -c 'rosdep install --from-paths src --ignore-src -r -y --rosdistro rolling'
RUN /bin/bash -c 'source /root/src/spaceros_ws/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'
# RUN rosinstall_generator --rosdistro rolling --from-path ros_gz --deps --format repos

# Build demos
WORKDIR ${WORKSPACE}
# RUN /bin/bash -c 'source /root/src/depends_ws/install/setup.bash'

COPY ./demo_entrypoint.sh /
ENTRYPOINT ["/demo_entrypoint.sh"]
CMD ["bash"]



