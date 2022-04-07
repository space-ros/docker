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
#
# Launch the container:
#   docker run -it --gpus all --net=host -e DISPLAY -v /tmp/.X11-unix openrobotics/moveit2_tutorials
#
# Run xauth add inside the container (get the values from the host system via 'xauth list'):
#   xauth add <hostname>/unix:1  MIT-MAGIC-COOKIE-1  <cookie-value>
#
# Launch the tutorials:
#   ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=true

FROM openrobotics/moveit2:latest

ARG VCS_REF
ARG VERSION="preview"

ARG MOVEIT2_WORKSPACE=/root/src/moveit2_ws
ARG MOVEIT2_UNDERLAY_WS=/root/src/moveit2_underlay_ws


# LABEL the image
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Space ROS"
LABEL org.label-schema.description="Preview version of the Space ROS/MoveIt2 platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker-images"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Get the MoveIt2 Tutorials source code
RUN mkdir -p ${MOVEIT2_WORKSPACE}/src
WORKDIR ${MOVEIT2_WORKSPACE}
RUN git clone https://github.com/ros-planning/moveit2_tutorials.git -b main src/moveit2_tutorials

# Get the repositories required by the MoveIt2 tutorials, but not included in Space ROS or MoveIt2
COPY moveit2_tutorials.repos ./
RUN vcs import src < moveit2_tutorials.repos

# Build it
RUN /bin/bash -c 'source ${MOVEIT2_UNDERLAY_WS}/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Set up the entrypoint
COPY ./moveit2_tutorials_entrypoint.sh /
ENTRYPOINT ["/moveit2_tutorials_entrypoint.sh"]
CMD ["bash"]
