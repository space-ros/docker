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
#ARG NAV2_BRANCH=jazzy #does not work on humble based spaceROS
RUN sudo git clone --branch $NAV2_BRANCH https://github.com/ros-navigation/navigation2.git

# to exclude gazebo_ros_pkgs->gazebo(classic) from depenencies
RUN sudo rm -rf navigation2/nav2_system_tests

# Get keys for Nav2 dependencies
WORKDIR ${NAVIGATION2_WS}/
SHELL ["/bin/bash", "-c"]

RUN source ${SPACEROS_DIR}/install/setup.bash && \
    sudo apt update && \
    rosdep keys -y \
      --from-paths src \
      --ignore-src     \
      --skip-keys "nav2_system_tests" \
      --rosdistro ${ROS_DISTRO} > ${NAVIGATION2_WS}/nav2_dep_keys.txt

# this is the keys that will be installed
#angles
#behaviortree_cpp_v3
#benchmark
#bond
#bondcpp
#cv_bridge
#diagnostic_updater
#eigen
#graphicsmagick
#image_transport
#laser_geometry
#libboost-program-options
#libboost-program-options-dev
#libceres-dev
#libomp-dev
#libqt5-core
#libqt5-gui
#libqt5-opengl
#libqt5-widgets
#map_msgs
#nlohmann-json-dev
#ompl
#python3-pytest
#python3-yaml
#qtbase5-dev
#resource_retriever
#!!rviz_common
#!!rviz_default_plugins -- this brings ignition-math6 for whatever reason. and required for nav2_rviz_plugins which is not optional
#!!rviz_ogre_vendor
#!!rviz_rendering
#slam_toolbox
#xsimd
#xtensor
#yaml_cpp_vendor



# Get rosinstall_generator
RUN sudo apt-get update -y && \
    sudo apt-get install -y python3-rosinstall-generator

# Clone Space ROS sources temporarily as input to rosinstall_generator
RUN mkdir ${SPACEROS_DIR}/src && \
    vcs import ${SPACEROS_DIR}/src < ${SPACEROS_DIR}/exact.repos

# Generate repos file for nav2 dependencies, exclude packages from Space ROS src
RUN rosinstall_generator \
  --rosdistro ${ROS_DISTRO} \
  --deps \
  --exclude-path ${SPACEROS_DIR}/src \
  -- $(cat ${NAVIGATION2_WS}/nav2_dep_keys.txt) \
  > ${NAVIGATION2_WS}/nav2_deps.repos

# throws this during execution, which is fine:
#The following unreleased packages/stacks will be ignored: --, benchmark, eigen, graphicsmagick, libboost-program-options, libboost-program-options-dev, libceres-dev, libomp-dev, libqt5-core, libqt5-gui, libqt5-opengl, libqt5-widgets, nlohmann-json-dev, python3-pytest, python3-yaml, qtbase5-dev, xsimd, xtensor
#The following unreleased '--exclude' packages/stacks will be ignored: ament_cmake_cobra, ament_cmake_ikos, ament_cobra, ament_ikos, cobra_vendor, process_sarif, rosidl_typesupport_introspection_tests, test_launch_ros, test_launch_testing, test_osrf_testing_tools_cpp, test_rmw_implementation, test_tf2, test_tracetools, test_tracetools_launch

# nav2_deps.repos includes:
#ignition_cmake2_vendor
#ignition_math6_vendor
# some nav2 pkgs. will remove after rosdep below


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

RUN source ${SPACEROS_DIR}/install/setup.bash && \
    sudo apt update && \
    rosdep install -y -s \
      --from-paths src \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} \
      --skip-keys "composition demo_nodes_py ikos \
                   lifecycle rmw_connextdds \
                   rmw_fastrtps_dynamic_cpp rmw_fastrtps_cpp \
                   rosidl_typesupport_fastrtps_c rosidl_typesupport_fastrtps_cpp \
                   urdfdom_headers \
                   " > rosdep-commands-nav2-deps.sh && \
    chmod u+x rosdep-commands-nav2-deps.sh && \
    ./rosdep-commands-nav2-deps.sh

# remove nav2 pkgs from deps. 
# do it after rosdep otherwise rosdep will bring them as ros-nav2*!
RUN sudo rm -rf src/navigation2

# this is what this rosdep would do, for documentation purposes
#  sudo -H apt-get install -y doxygen
#  sudo -H apt-get install -y graphicsmagick-libmagick-dev-compat
#  sudo -H apt-get install -y libassimp-dev
#  sudo -H apt-get install -y libboost-all-dev
#  sudo -H apt-get install -y libboost-coroutine1.74.0
#  sudo -H apt-get install -y libboost-coroutine-dev
#  sudo -H apt-get install -y libboost-program-options1.74.0
#  sudo -H apt-get install -y libboost-program-options-dev
#  sudo -H apt-get install -y libboost-python1.74.0
#  sudo -H apt-get install -y libboost-python-dev
#  sudo -H apt-get install -y libceres-dev
#  sudo -H apt-get install -y libcurl4-openssl-dev
#  sudo -H apt-get install -y libflann-dev
#  sudo -H apt-get install -y libfreetype6-dev
#  sudo -H apt-get install -y libgl1-mesa-dev
#  sudo -H apt-get install -y libglu1-mesa-dev
#  sudo -H apt-get install -y libgraphicsmagick++1-dev
#  sudo -H apt-get install -y libignition-cmake2-dev # cannot remove as a dependency for rvizXXX that is a proper dependency of nav2
#  sudo -H apt-get install -y libignition-math6-dev  # cannot remove as a dependency for rvizXXX that is a proper dependency of nav2
#  sudo -H apt-get install -y liblapack-dev
#  sudo -H apt-get install -y libode-dev
#  sudo -H apt-get install -y libopencv-dev
#  sudo -H apt-get install -y libqt5core5a
#  sudo -H apt-get install -y libqt5gui5
#  sudo -H apt-get install -y libqt5opengl5
#  sudo -H apt-get install -y libqt5svg5
#  sudo -H apt-get install -y libqt5svg5-dev
#  sudo -H apt-get install -y libqt5widgets5
#  sudo -H apt-get install -y libsuitesparse-dev
#  sudo -H apt-get install -y libx11-dev
#  sudo -H apt-get install -y libxaw7-dev
#  sudo -H apt-get install -y libxrandr-dev
#  sudo -H apt-get install -y libyaml-cpp-dev
#  sudo -H apt-get install -y libzmq3-dev
#  sudo -H apt-get install -y python3-opencv
#  sudo -H apt-get install -y qtbase5-dev
#  sudo -H apt-get install -y uuid-dev



# Build dependencies
RUN source ${SPACEROS_DIR}/install/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON




# Install remaining nav2 dependencies (not in rosdistro)
WORKDIR ${NAVIGATION2_WS}
SHELL ["/bin/bash", "-c"]
RUN source ${NAV2_DEPS_WS}/install/setup.bash && \
    sudo apt update && \
    rosdep install -y -s \
      --from-paths src \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} \
      --skip-keys "composition demo_nodes_py ikos \
                   lifecycle rmw_connextdds \
                   rmw_fastrtps_dynamic_cpp rmw_fastrtps_cpp \
                   rosidl_typesupport_fastrtps_c rosidl_typesupport_fastrtps_cpp \
                   urdfdom_headers \
                   " > rosdep-commands-nav2.sh && \
    chmod u+x rosdep-commands-nav2.sh && \
    ./rosdep-commands-nav2.sh

# this is what this rosdep would do, for documentation purposes
#  sudo -H apt-get install -y libomp-dev
#  sudo -H apt-get install -y libxtensor-dev
#  sudo -H apt-get install -y nlohmann-json3-dev
#  sudo -H apt-get install -y ros-humble-ompl   # this is ok, it does not bring entire ros-* 
# afterwards dpkg lists ros-humble-ompl and ros-humble-ros-workspace
# dpkg also shows the following: libignition-cmake2-dev libignition-math6 libignition-math6-dev



# Build Navigation2
SHELL ["/bin/bash", "-c"]
RUN source ${NAV2_DEPS_WS}/install/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON


# Install rviz2 to send goals to Nav2
# this is not a good way to do this. will be done from source later
#RUN sudo apt update && sudo apt install -y ros-humble-rviz2

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
