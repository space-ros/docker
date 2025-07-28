#!/bin/bash
set -e

# Setup the MoveIt2 environment
source "${MOVEIT2_DIR}/install/setup.bash"

exec "$@"
