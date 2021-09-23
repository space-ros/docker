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
#   WORKSPACE   - The location for the Space ROS source code in the container (default: /usr/local/src/spaceros_ws)
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
ARG WORKSPACE=/usr/local/src/moveit2_ws

# LABEL the image
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Space ROS"
LABEL org.label-schema.description="Preview version of the Space ROS/MoveIt2 platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://openrobotics.org"
LABEL org.label-schema.vcs-url="https://github.com/ros2/ros2/tree/spaceros"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Get the MoveIt2 Tutorials source code
WORKDIR ${WORKSPACE}/src
RUN git clone https://github.com/ros-planning/moveit2_tutorials.git
RUN vcs import < moveit2_tutorials/moveit2_tutorials.repos

# Build it
WORKDIR ${WORKSPACE}
RUN /bin/bash -c 'source /usr/local/src/spaceros_ws/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Set up the entrypoint
COPY ./moveit2_tutorials_entrypoint.sh /
ENTRYPOINT ["/moveit2_tutorials_entrypoint.sh"]
CMD ["bash"]
