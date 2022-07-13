#!/bin/bash
set -e

# Setup the Demo environment
source /root/src/spaceros_ws/install/setup.bash
source /root/src/depends_ws/install/setup.bash
exec "$@"
