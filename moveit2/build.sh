#!/usr/bin/env bash

ORG=openrobotics
IMAGE=moveit2
TAG=latest

VCS_REF=""
VERSION=preview

# Exit script with failure if build fails
set -eo pipefail

echo ""
echo "##### Building Space ROS/MoveIt2 Docker Image #####"
echo ""

docker build -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" \
    --build-arg SPACE_ROS_IMAGE="${SPACE_ROS_IMAGE:-osrf/space-ros:humble-2024.10.0}" \
    .

echo ""
echo "##### Done! #####"

