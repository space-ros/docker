#!/bin/bash
set -e

# Setup the Demo environment
source /home/spaceros-user/src/mars_rover/install/setup.bash
exec "$@"
