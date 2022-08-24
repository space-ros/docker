#! /usr/bin/env bash

ORG=openrobotics
IMAGE=rtems
TAG=latest

VCS_REF=""
VERSION=preview


echo ""
echo "##### Building RTEMS Docker Image #####"

docker build -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" .

echo ""
echo "##### Done! #####"

