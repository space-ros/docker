#!/bin/bash
set -e

# Setup the Navigation2 environment
source "/home/spaceros-user/src/navigation2/install/setup.bash"
exec "$@"
