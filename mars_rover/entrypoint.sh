#!/bin/bash
set -e

# Setup the Demo environment
source "${MARS_ROVER_DEMO_DIR}/install/setup.bash"
exec "$@"
