#!/bin/bash
SPACE_ROS_IMAGE="${SPACE_ROS_IMAGE:-osrf/space-ros:latest}"

docker run --rm \
    -v ./:/home/spaceros-user/mount/ \
    -w /home/spaceros-user/mount/ \
    $SPACE_ROS_IMAGE \
    bash -c './update_nav2_repos.sh'
