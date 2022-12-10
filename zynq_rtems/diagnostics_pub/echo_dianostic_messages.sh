#!/bin/bash

# probably best to use RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# since that is what is being sent from the zenoh-dds bridge
ros2 topic echo /diagnostics diagnostic_msgs/DiagnosticArray
