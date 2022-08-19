#!/bin/bash
set -e

# Setup the Demo environment
source /root/src/spaceros_demo/install/setup.bash
exec "$@"
