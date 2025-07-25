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

FROM openrobotics/moveit2:latest

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

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Update the ROS package keys
ADD --chmod=644 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros-archive-keyring.gpg

# Install base image dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update -y && \
    sudo -E apt-get install -y \
      git-lfs \
      python3-rosinstall-generator

RUN sudo git clone https://github.com/space-ros/demos.git ${DEMO_DIR}/src/demos
RUN sudo git clone https://github.com/space-ros/simulation.git ${DEMO_DIR}/src/simulation
RUN sudo chown -R ${USERNAME}:${USERNAME} ${DEMO_DIR}

# Generate repos file for demo dependencies, excluding packages from Space ROS core.
COPY --chown=${USERNAME}:${USERNAME} demo-pkgs.txt /tmp/
COPY --chown=${USERNAME}:${USERNAME} excluded-pkgs.txt /tmp/
RUN rosinstall_generator \
  --rosdistro ${ROS_DISTRO} \
  --deps \
  --exclude-path ${SPACEROS_DIR}/src ${MOVEIT2_DIR}/src \
  --exclude $(cat /tmp/excluded-pkgs.txt) -- \
  -- $(cat /tmp/demo-pkgs.txt) \
  > /tmp/demo_generated_pkgs.repos

WORKDIR ${DEMO_DIR}
RUN vcs import src < /tmp/demo_generated_pkgs.repos

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y \
&& /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' \
&& /bin/bash -c 'source "${MOVEIT2_DIR}/install/setup.bash"' \
&& rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# Build the demo
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash && source ${MOVEIT2_DIR}/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Add the user to the render group so that the user can access /dev/dri/renderD128
RUN sudo usermod -aG render $USERNAME

# Setup the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
