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

FROM osrf/space-ros:latest

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Navigation2"
LABEL org.label-schema.description="Preview version of the Navigation2/Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker-images"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Define workspace locations
ENV NAVIGATION2_WS=${HOME_DIR}/nav2_ws
ENV NAV2_DEPS_WS=${HOME_DIR}/nav2_deps_ws

# Get the Navigation2 source code
RUN mkdir -p ${NAVIGATION2_WS}/src
WORKDIR ${NAVIGATION2_WS}/src
ARG NAV2_BRANCH=humble
RUN sudo git clone --branch $NAV2_BRANCH https://github.com/ros-planning/navigation2.git

# Get keys for Nav2 dependencies
WORKDIR ${NAVIGATION2_WS}/
SHELL ["/bin/bash", "-c"]
RUN source ${SPACEROS_DIR}/install/setup.bash && sudo apt update && rosdep keys --from-paths src --ignore-src --rosdistro humble -y > ${NAVIGATION2_WS}/nav2_dep_keys.txt

# Get rosinstall_generator
RUN sudo apt-get update -y && sudo apt-get install -y python3-rosinstall-generator

# Clone Space ROS sources temporarily as input to rosinstall_generator
RUN mkdir ${SPACEROS_DIR}/src \
  && vcs import ${SPACEROS_DIR}/src < ${SPACEROS_DIR}/exact.repos

# Generate repos file for nav2 dependencies, exclude packages from Space ROS src
RUN rosinstall_generator \
  --rosdistro ${ROSDISTRO} \
  --deps \
  --exclude-path ${SPACEROS_DIR}/src -- \
  -- $(cat ${NAVIGATION2_WS}/nav2_dep_keys.txt) \
  > ${NAVIGATION2_WS}/nav2_deps.repos

# Remove unneeded src files now that repos are generated
RUN rm -rf ${SPACEROS_DIR}/src

# Get the repositories required by Nav2
RUN mkdir -p ${NAV2_DEPS_WS}/src
WORKDIR ${NAV2_DEPS_WS}
RUN sudo vcs import src < ${NAVIGATION2_WS}/nav2_deps.repos

# Update the ownership of the source files
RUN sudo chown -R ${USERNAME}:${USERNAME} ${NAV2_DEPS_WS}

# Install nav2_deps_ws dependencies
SHELL ["/bin/bash", "-c"]
RUN source ${SPACEROS_DIR}/install/setup.bash && sudo apt update && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
      --skip-keys "composition demo_nodes_py ikos lifecycle rmw_connextdds rmw_fastrtps_dynamic_cpp rmw_fastrtps_cpp \
                   rosidl_typesupport_fastrtps_c rosidl_typesupport_fastrtps_cpp urdfdom_headers"

# Build dependencies
RUN source ${SPACEROS_DIR}/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Install remaining nav2 dependencies (not in rosdistro)
WORKDIR ${NAVIGATION2_WS}
SHELL ["/bin/bash", "-c"]
RUN source ${NAV2_DEPS_WS}/install/setup.bash && sudo apt update && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
      --skip-keys "composition demo_nodes_py ikos lifecycle rmw_connextdds rmw_fastrtps_dynamic_cpp rmw_fastrtps_cpp \
                   rosidl_typesupport_fastrtps_c rosidl_typesupport_fastrtps_cpp urdfdom_headers"

# Build Navigation2
SHELL ["/bin/bash", "-c"]
RUN source ${NAV2_DEPS_WS}/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
