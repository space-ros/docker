#!/usr/bin/env bash

ORG=osrf
IMAGE=space_nav2
TAG=latest

VCS_REF=""
VERSION=preview

# Exit script with failure if build fails
set -eo pipefail

echo ""
echo "##### Building Navigation2/Space ROS Docker Image #####"
echo ""

docker build -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" \
    --build-arg SPACE_ROS_IMAGE="${SPACE_ROS_IMAGE:-osrf/space-ros:jazzy-2025.07.0}" \
    .

echo ""
echo "##### Done! #####"

