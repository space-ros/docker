#!/bin/bash

BASE_IMAGE_NAME=${1:-"osrf/space-ros:latest"}
docker run --rm \
    -v ./:/home/spaceros-user/mount/ \
    -w /home/spaceros-user/mount/ \
    $BASE_IMAGE_NAME \
    bash -c './update_nav2_repos.sh'
