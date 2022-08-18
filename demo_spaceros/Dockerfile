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

FROM openrobotics/spaceros:latest

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
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker-images"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Define a few key variables
ARG SPACEROS_DEMO_DIR=${SRC_DIR}/spaceros_demo
ARG ROSDISTRO=rolling
ENV IGNITION_VERSION fortress

# Get rosinstall_generator
RUN sudo apt-get update -y && sudo apt-get install -y python3-rosinstall-generator

# Generate repos file for demo dependencies, excluding packages from Space ROS core.
RUN mkdir -p ${SPACEROS_DEMO_DIR}/src
WORKDIR ${SPACEROS_DEMO_DIR}
COPY --chown=${USERNAME}:${USERNAME} spaceros-demo-pkgs.txt /tmp/
COPY --chown=${USERNAME}:${USERNAME} excluded-pkgs.txt /tmp/
RUN rosinstall_generator \
  --rosdistro ${ROSDISTRO} \
  --upstream-development \
  --deps \
  --exclude-path ${SPACEROS_DIR}/src \
  --exclude $(cat /tmp/excluded-pkgs.txt) -- \
  $(cat /tmp/spaceros-demo-pkgs.txt) \
  > /tmp/spaceros_demo_generated_pkgs.repos

# Get the source for the dependencies
RUN vcs import src < /tmp/spaceros_demo_generated_pkgs.repos
COPY --chown=${USERNAME}:${USERNAME} spaceros_demo_manual_pkgs.repos /tmp/
RUN vcs import src < /tmp/spaceros_demo_manual_pkgs.repos --force

RUN sudo apt-get update -y
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the demo
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Add the user to the render group
RUN sudo usermod -aG render $USERNAME

# Setup the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
