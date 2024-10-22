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
# A Docker configuration script to build the Navigation2/Space ROS image.
#
# The script provides the following build arguments:
#
#   VCS_REF     - The git revision of the Space ROS source code (no default value).
#   VERSION     - The version of Space ROS (default: "preview")

FROM osrf/space_nav2:latest

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Navigation2 demo"
LABEL org.label-schema.description="Preview version of the Navigation2/Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Define workspace locations
ENV NAVIGATION2_WS=${HOME_DIR}/nav2_ws
ENV NAV2_DEMO_WS=${HOME_DIR}/nav2_demo_ws

# Install LFS for simulation resources
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update -y && \
    sudo apt-get install -y \
      git-lfs

# Grab required simulation resources
WORKDIR ${NAV2_DEMO_WS}
RUN mkdir -p src
COPY nav2_demo.repos /tmp/
RUN vcs import --input /tmp/nav2_demo.repos src/

# Install them
RUN source ${SPACEROS_DIR}/install/setup.bash && \
    source ${NAVIGATION2_WS}/install/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN rm -rf src build log

# Setup mars_rover demo
# Install map file for localization
COPY --chown=${USERNAME}:${USERNAME}  mars_map* .

# Install nav2 config file for mars rover demo
COPY --chown=${USERNAME}:${USERNAME}  nav2_params.yaml .

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
