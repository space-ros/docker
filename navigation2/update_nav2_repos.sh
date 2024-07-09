#!/bin/bash

source $SPACEROS_DIR/install/setup.bash

# TODO - replace ./generate_repos.sh with file from the spaceros image as soon as the PR is merged:
# https://github.com/space-ros/space-ros/pull/164

./generate_repos.sh \
    --rosdistro $ROSDISTRO \
    --packages navigation2-pkgs.txt \
    --excluded-packages excluded-pkgs.txt \
    --outfile navigation2.repos \
    --upstream false # this allows us to select specific sub-packages from a single repo
