#!/bin/bash
set -e

# Setup the Demo environment
source /home/spaceros-user/src/spaceros_demo/install/setup.bash
exec "$@"
