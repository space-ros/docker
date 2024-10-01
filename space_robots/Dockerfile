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

#FROM openrobotics/moveit2:latest
FROM moveit/moveit2:humble-source

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Curiosity Rover"
LABEL org.label-schema.description="Curiosity rover demo on the Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Define a few key variables
ENV DEMO_DIR=${HOME_DIR}/demos_ws
ENV IGNITION_VERSION fortress
ENV GZ_VERSION fortress
#ENV SPACEROS_DIR=/opt/spaceros

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Install base image dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update -y && \
    sudo apt-get install -y \
      git-lfs \
      python3-rosinstall-generator

# TODO(anyone): remove demo-pkgs.txt, no packages left after exclusions
# Generate repos file for demo dependencies, excluding packages from Space ROS core.
# COPY --chown=${USERNAME}:${USERNAME} demo-pkgs.txt /tmp/
# COPY --chown=${USERNAME}:${USERNAME} excluded-pkgs.txt /tmp/
# RUN rosinstall_generator \
#   --rosdistro ${ROSDISTRO} \
#   --deps \
#   --exclude-path ${SPACEROS_DIR}/src \
#   --exclude-path ${MOVEIT2_DIR}/src \
#   --exclude $(cat /tmp/excluded-pkgs.txt) -- \
#   -- $(cat /tmp/demo-pkgs.txt) \
#   > /tmp/demo_generated_pkgs.repos

RUN mkdir -p ${DEMO_DIR}/src
WORKDIR ${DEMO_DIR}


# Install libmongoc for development
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get install libmongoc-dev -y

# Compile mongo cxx driver https://mongocxx.org/mongocxx-v3/installation/linux/
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get install libssl-dev build-essential devscripts debian-keyring fakeroot debhelper cmake libboost-dev libsasl2-dev libicu-dev libzstd-dev doxygen wget -y

RUN wget https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.6.7/mongo-cxx-driver-r3.6.7.tar.gz
RUN tar -xzf mongo-cxx-driver-r3.6.7.tar.gz
RUN cd mongo-cxx-driver-r3.6.7/build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && sudo cmake --build . --target EP_mnmlstc_core && cmake --build . && sudo cmake --build . --target install

# Get the source for the dependencies
# RUN vcs import src < /tmp/demo_generated_pkgs.repos
ENV SPACEROS_DIR=/root/ws_moveit
ENV MOVEIT2_DIR=${SPACEROS_DIR}

RUN echo "SPACEROS_DIR is set to: ${SPACEROS_DIR}"

COPY --chown=${USERNAME}:${USERNAME} demo_manual_pkgs.repos /tmp/
RUN vcs import src < /tmp/demo_manual_pkgs.repos
RUN /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' 

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y 

RUN echo "MOVEIT2_DIR is set to: ${MOVEIT2_DIR}"

RUN /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' 
RUN /bin/bash -c 'source "${MOVEIT2_DIR}/install/setup.bash"' 

RUN sudo apt-get update && sudo apt-get install -y curl gnupg2 lsb-release \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add - \
  && sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
  && sudo apt-get update

RUN sudo apt-get update && sudo apt-get install -y vim python3-pip


ENV ROSDISTRO=humble
RUN rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROSDISTRO}

# Build the demo
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash && source ${MOVEIT2_DIR}/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

ENV USERNAME=root 

RUN echo "USERNAME is set to: $USERNAME"


# Add the user to the render group so that the user can access /dev/dri/renderD128
RUN sudo usermod -aG render $USERNAME

# Create /demo_ws/ directory and copy new files
RUN mkdir -p /demos_ws/
COPY generate_src.py /demos_ws/
COPY sample_code_by_gpt.py /demos_ws/
COPY sample_code_turn_left_by_gpt.py /demos_ws/
COPY config.json /demos_ws/
COPY prompt.txt /demos_ws/ 

# Grant execute permissions to Python scripts (if needed)
RUN chmod +x /demos_ws/generate_src.py /demos_ws/sample_code_by_gpt.py /demos_ws/sample_code_turn_left_by_gpt.py

RUN pip install openai

# Setup the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
