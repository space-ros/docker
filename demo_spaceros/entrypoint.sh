#!/bin/bash
set -e

# Setup the Demo environment
source /root/src/spaceros_demo_ws/install/setup.bash
exec "$@"
