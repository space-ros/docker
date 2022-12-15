#!/bin/bash

# probably best to use RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# since that is what is being sent from the zenoh-dds bridge
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=44
ros2 topic echo /diagnostics diagnostic_msgs/DiagnosticArray
