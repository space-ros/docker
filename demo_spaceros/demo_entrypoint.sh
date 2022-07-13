#!/bin/bash
set -e

<<<<<<< HEAD
# Setup the MoveIt2 environment
source "/root/src/spaceros_demo_ws/install/setup.bash"
exec "$@"
=======
# Setup the Demo environment
source /root/src/spaceros_ws/install/setup.bash
source /root/src/depends_ws/install/setup.bash
exec "$@"
>>>>>>> tonylitianyu/demo_depends
