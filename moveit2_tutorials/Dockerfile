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

# LABEL the image
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="MoveIt2 Tutorials"
LABEL org.label-schema.description="MoveIt2 Tutorials on the Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker-images"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Define key locations
ARG WORKSPACE=/root/src/moveit2_ws
WORKDIR ${WORKSPACE}

# Get the MoveIt2 Tutorials source code
RUN mkdir -p ${WORKSPACE}/src
COPY moveit2_tutorials.repos ./
RUN git clone https://github.com/ros-planning/moveit2_tutorials.git -b main src/moveit2_tutorials
RUN cd src/moveit2_tutorials && git checkout c98d7f10aefb8dffde60621ec5b72d1f5d17caaf

# Get the repositories required by MoveIt2, but not included in Space ROS
RUN vcs import src < moveit2_tutorials.repos

# Build it
RUN /bin/bash -c 'source /root/src/spaceros_ws/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
