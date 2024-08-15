#!/bin/bash

source $SPACEROS_DIR/install/setup.bash
SCRIPT_PATH="$SPACEROS_DIR/scripts"
bash $SCRIPT_PATH/generate-repos.sh \
    --rosdistro $ROS_DISTRO \
    --packages navigation2-pkgs.txt \
    --excluded-packages excluded-pkgs.txt \
    --outfile navigation2.repos \
    --upstream false # this allows us to select specific sub-packages from a single repo
