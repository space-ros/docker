#!/bin/bash

# Setup the MoveIt2 environment
source "${MOVEIT2_DIR}/install/setup.bash"

exec "$@"
