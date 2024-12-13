# RACS2 on Space ROS and Space Robots Demo Docker Image

The RACS2 on Space ROS and Space Robots Demo docker image uses the space_robots demo docker image (*openrobotics/space_robots_demo:latest*) as its base image.
Build instructions for that image can be found in [this README](../space_robots/README.md).
The Dockerfile installs all of the prerequisite system dependencies along with the demos source code, then builds the Space ROS Space Robots and [RACS2](https://github.com/jaxa/racs2_bridge) demo.


This is RACS2 Bridge demo for Curiosity Mars rover.

## Building the Demo Docker

The demo image builds on top of the `spaceros`, `moveit2`, `space_robots` images.
To build the docker image, first ensure the `spaceros` base image is available either by [building it locally](https://github.com/space-ros/space-ros) or pulling it.

Then build the `moveit2`, `space_robots` and `racs2_demos_on_spaceros` demo images: 

```bash
cd ../moveit2
./build.sh
cd ../space_robots
./build.sh
cd ../racs2_demos_on_spaceros
./build.sh
```

## Running the Demo Docker

run the following to allow GUI passthrough:
```bash
xhost +local:docker
```

Then run:
```bash
./run.sh
```

Depending on the host computer, you might need to remove the ```--gpus all``` flag in ```run.sh```, which uses your GPUs.

## Running the Demos

### Curiosity Mars rover demo
Launch the rover demo:
```bash
ros2 launch mars_rover mars_rover.launch.py
```

#### RACS2 Bridge demo

##### Running racs2 bridge node
Open a new terminal and attach to the currently running container:

```bash
docker exec -it <container-name> bash
source install/setup.bash
ros2 run bridge_py_s bridge_py_s_node  --ros-args --params-file ./src/bridge_py_s/config/params.yaml
```

##### Running cFS bridge app & run_app app
Open a new terminal and attach to the currently running container:

```bash
docker exec -it <container-name> bash
cd ~/racs2_ws
cd cfs/build/exe/cpu1/
./core-cpu1
```

**Executing commands to the rover must be done with this terminal active.**


#### Available Commands

Drive commands to the rover are input via keyboard. The keymap is as follows.

* "w": Drive the rover forward
* "s": Drive the rover backward
* "a": Turn left
* "d": Turn right
* "x": Stop the rover

#### Nodes in Demo 
<img width="899" alt="racs2_demo_on_spaceros_nodes" src="https://github.com/user-attachments/assets/77bfdc70-4a5f-4124-a526-ba746db44ec3" />

## Reference

* [Hiroki Kato, et al. "ROS and cFS System (RACS): Easing Space Robotic Development post-opensource activities and ROS2 integration" Flight Software Workshop 2021.](https://drive.google.com/file/d/11L48doT_pRNs7R0hdChPALqJO849TvV2/view?usp=drive_web)
