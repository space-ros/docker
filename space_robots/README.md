# Space ROS Space Robots Demo Docker Image

The Space ROS Space Robots Demo docker image uses the moveit2 docker image (*openrobotics/moveit2:latest*) as its base image.
Build instructions for that image can be found in [this README](../moveit2/README.md).
The Dockerfile installs all of the prerequisite system dependencies along with the demos source code, then builds the Space ROS Space Robots Demo.

This is for Curiosity Mars rover and Canadarm demos.

## Building the Demo Docker

The demo image builds on top of the `spaceros` and `moveit2` images.
To build the docker image, first ensure the `spaceros` base image is available either by [building it locally](https://github.com/space-ros/space-ros) or pulling it.

Then build the `moveit2` and the `space_robots` demo images:

```bash
cd ../moveit2
./build.sh
cd ../space_robots
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
Launch the demo:
```bash
ros2 launch mars_rover mars_rover.launch.py
```

On the top left corner, click on the refresh button to show camera feed.

### Perform Tasks

#### Setup

Open a new terminal and attach to the currently running container:

```bash
docker exec -it <container-name> bash
```

Make sure packages are sourced:

```bash
source ~/spaceros/install/setup.bash
```

```bash
source ~/demos_ws/install/setup.bash
```

#### Available Commands

Drive the rover forward

```bash
ros2 service call /move_forward std_srvs/srv/Empty
```

Stop the rover

```bash
ros2 service call /move_stop std_srvs/srv/Empty
```

Turn left

```bash
ros2 service call /turn_left std_srvs/srv/Empty
```

Turn right

```bash
ros2 service call /turn_right std_srvs/srv/Empty
```

Open the tool arm:

```bash
ros2 service call /open_arm std_srvs/srv/Empty
```

Close the tool arm:

```bash
ros2 service call /close_arm std_srvs/srv/Empty
```

Open the mast (camera arm)

```bash
ros2 service call /mast_open std_srvs/srv/Empty
```

Close the mast (camera arm)

```bash
ros2 service call /mast_close std_srvs/srv/Empty
```

##### Generate codes for moving rover by OpenAI API

Step.1 Initial settingsi

Check the config.json for input your API-Key and the LLM model you want to

Step.2 After run the launch demo command, run the code

```bash
python3 generate_src.py
```
("openai" of python module is verified in the version 1.44.0)


Step.3 Automatically saving the generated code by LLM

```bash
gen_script_20240907_0835.py
```

#### Canadarm demo

```bash
ros2 launch canadarm canadarm.launch.py
```
