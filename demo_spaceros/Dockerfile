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

# Get the depends source code
WORKDIR ${WORKSPACE}/..
RUN mkdir -p depends_ws/src

# # Get the repositories, but not included in Space ROS
WORKDIR /root/src/depends_ws
COPY demo_spaceros.repos ./
RUN vcs import src < demo_spaceros.repos

COPY addition.repos ./
RUN vcs import src < addition.repos

RUN apt-get update -y
RUN rosdep install --from-paths src --ignore-src -r -y

ENV IGNITION_VERSION fortress

# Set up the entrypoint
WORKDIR /root/src/depends_ws
# RUN /bin/bash -c 'rosdep install --from-paths src --ignore-src -r -y --rosdistro rolling'
RUN /bin/bash -c 'source /root/src/spaceros_ws/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'
# RUN rosinstall_generator --rosdistro rolling --from-path ros_gz --deps --format repos

# Build demos
WORKDIR ${WORKSPACE}

RUN rm /ros_entrypoint.sh
COPY ./demo_entrypoint.sh /
ENTRYPOINT ["/demo_entrypoint.sh"]
CMD ["bash"]



