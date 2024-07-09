#!/bin/bash
set -e

# Setup the Navigation2 environment
source "$NAVIGATION2_WS/install/setup.bash"
exec "$@"

