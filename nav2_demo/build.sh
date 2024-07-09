#!/usr/bin/env bash

ORG=osrf
SPACENAV_IMAGE=space_nav2_demo
RVIZ2_IMAGE=rviz2_space_nav2_demo
TAG=latest

VCS_REF=""
SPACENAV_VERSION=preview

# Exit script with failure if build fails
set -eo pipefail

echo ""
echo "##### Building Navigation2/Space ROS Docker Image #####"
echo ""

docker build \
    -t $ORG/$SPACENAV_IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg SPACENAV_VERSION="$SPACENAV_VERSION" .

echo ""
echo "##### Done building Space Navigation2 Docker Image! #####"
echo ""

BASE_ROSDISTRO=$(docker run --rm $ORG/$SPACENAV_IMAGE:$TAG bash -c 'echo "$ROSDISTRO"')

echo ""
echo "##### Building additional ROS2 $BASE_ROSDISTRO image with Rviz2 and Nav2, for visualization and goal setting  #####"
echo ""

docker build -f rviz_nav2.Dockerfile \
    --build-arg BASE_ROSDISTRO=$BASE_ROSDISTRO \
    -t $ORG/$RVIZ2_IMAGE:$TAG \
    .

echo ""
echo "##### Done building Rviz2 Image! #####"
echo ""
