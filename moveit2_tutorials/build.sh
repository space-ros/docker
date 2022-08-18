#!/usr/bin/env bash

ORG=openrobotics
IMAGE=moveit2_tutorials
TAG=latest

VCS_REF=""
VERSION=preview

echo ""
echo "##### Building MoveIt2 Tutorials Docker Image #####"
echo ""

docker build -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" .

echo ""
echo "##### Done! #####"

