Space ROS Docker Image Templates
================================

The projects in this repository are applications and demonstrations built on the core [spaceros image](https://github.com/space-ros/space-ros).

See individual template directories for details.

* [moveit2](./moveit2)
* [nav2_demo](./nav2_demo)
* [navigation2](./navigation2)
* [renode_rcc](./renode_rcc)
* [rtems](./rtems)
* [space_robots](./space_robots)
* [zynq_rtems](./zynq_rtems)


### Build stacks

To build the docker images available in the stack, run the following command:

```bash
# To show the available options
./build.sh -help
# Usage: ./build.sh stack-name [options]
# USAGE
# -h, --help: display this help message
#   stack-name: the name of the stack to build. Must be one of the following:
#     - gui
#     - nav2
#     - moveit2

# To build GUI stack
./build.sh gui
```

### Starting a stack container

To run the docker images available in the stack, run the following command:

```bash
# To show the available options
./run.sh -help
# Usage: ./build.sh stack-name [options]
# USAGE
# -h, --help: display this help message
#   stack-name: the name of the stack to build. Must be one of the following:
#     - gui
#     - nav2
#     - moveit2

# To run GUI stack
./run.sh gui
```
