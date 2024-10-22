#!/bin/bash
set -e

# Setup the Navigation2 environment
source "${NAV2_DEMO_WS}/install/setup.bash"
exec "$@"
