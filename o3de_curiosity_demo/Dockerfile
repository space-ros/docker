# Copyright 2024 Azmyin Md. Kamal
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
# A Docker configuration script to build the o3de_curiosit_docker image that showcases Open 3D Engine
# in simulating a Curiositry rover traversing through a test ground inspired by the NASA JPL Mars Yard
#
# The script provides the following build arguments:
#
#   VCS_REF     - The git revision of the Space ROS source code (no default value).
#   VERSION     - The version of Space ROS (default: "preview")

FROM openrobotics/moveit2:latest

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="O3DE Curiosity Rover"
LABEL org.label-schema.description="Curiosity rover demo on a new test enviornment on the Space ROS platform and Open 3D Engine"
LABEL org.label-schema.vendor="Nasa Space ROS Sim Summer Spring Challenge 2024"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/Mechazo11/space-ros-docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Define a few key variables
ENV DEMO_DIR=${HOME_DIR}/demos_ws
ENV IGNITION_VERSION fortress
ENV GZ_VERSION fortress
ENV PROJECT_NAME=RobotSim
ENV O3DE_DIR=${HOME_DIR}/O3DE
ENV 03DE_EXTRAS=${HOME_DIR}/az-o3de-extras

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Get rosinstall_generator
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
# RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
#   --mount=type=cache,target=/var/lib/apt,sharing=locked \
#   sudo apt-get update -y && sudo apt-get install -y python3-rosinstall-generator

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y && sudo apt-get install -y \
  python3-rosinstall-generator \
  libboost-python-dev build-essential cmake libboost-all-dev libois-dev \
  libzzip-dev libfreeimage-dev libfreetype6-dev libx11-dev libxt-dev libxaw7-dev \
  libsdl2-dev libpugixml-dev libglew-dev qtbase5-dev libspdlog-dev libfmt-dev \
  libignition-gazebo6-dev libssl-dev devscripts debian-keyring fakeroot debhelper \
  libboost-dev libsasl2-dev libicu-dev libzstd-dev doxygen libtinyxml-dev \
  libtinyxml2-dev python3-pyqt5

RUN mkdir -p ${DEMO_DIR}/src
WORKDIR ${DEMO_DIR}

# Install libmongoc for development
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get install libmongoc-dev -y

# Compile mongo cxx driver https://mongocxx.org/mongocxx-v3/installation/linux/
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get install libssl-dev build-essential devscripts debian-keyring fakeroot debhelper cmake libboost-dev libsasl2-dev libicu-dev libzstd-dev doxygen -y
RUN wget https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.6.7/mongo-cxx-driver-r3.6.7.tar.gz
RUN tar -xzf mongo-cxx-driver-r3.6.7.tar.gz
RUN cd mongo-cxx-driver-r3.6.7/build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && sudo cmake --build . --target EP_mnmlstc_core && cmake --build . && sudo cmake --build . --target install

# Get the source for the dependencies
# RUN vcs import src < /tmp/demo_generated_pkgs.repos
COPY --chown=${USERNAME}:${USERNAME} o3de-robot-sim.repos /tmp/
RUN vcs import src < /tmp/o3de-robot-sim.repos && /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' 

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y \
&& /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' \
&& /bin/bash -c 'source "${MOVEIT2_DIR}/install/setup.bash"' \
&& rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROSDISTRO}

# Build the demo_ws
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash && source ${MOVEIT2_DIR}/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Setup enviornment variables


# Install Open 3D Engine


# Register Engine


# Create O3DE/Projects and O3DE/Gems directory

# Git clone RobotSim, NasaMarsRoverGem and MarsYardGem

# # Add the user to the render group so that the user can access /dev/dri/renderD128
# RUN sudo usermod -aG render $USERNAME

# # Setup the entrypoint
# COPY ./entrypoint.sh /
# ENTRYPOINT ["/entrypoint.sh"]
# CMD ["bash"]
