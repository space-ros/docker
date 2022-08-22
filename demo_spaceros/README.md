# Space ROS Demo Docker Image

The Space ROS Demo docker image uses the Space ROS docker image (*openrobotics/spaceros:latest*) as its base image. The Dockerfile installs all of the prerequisite system dependencies along with the demo source code, then builds the Space ROS Demo.

## Building the Demo Docker

To build the docker image, run:

```
$ ./build.sh
```

## Running the Demo Docker

run the following to allow GUI passthrough:
```
$ xhost +local:docker
```

Then run:
```
$ ./run.sh
```

Depending on the host computer, you might need to remove ```--gpus all``` flag in ```run.sh```, which uses your GPUs.

## Running the Demo

Launch the demo:
```
$ ros2 launch mars_rover mars_rover.launch.py
```

On the top left corner, click on the refresh button to show camera feed.

## Perform Tasks

### Setup

Open a new terminal and attach to the currently running container:

```
$ docker exec -it <container-name> bash
```

Make sure packages are sourced:

```
$ source ~/src/spaceros/install/setup.bash
```

```
$ source ~/src/spaceros_demo/install/setup.bash
```

### Available Commands

Drive the rover forward

```
$ ros2 service call /move_forward std_srvs/srv/Empty
```

Stop the rover

```
$ ros2 service call /move_stop std_srvs/srv/Empty
```

Turn left

```
$ ros2 service call /turn_left std_srvs/srv/Empty
```

Turn right

```
$ ros2 service call /turn_right std_srvs/srv/Empty
```

Open the tool arm:

```
$ ros2 service call /open_arm std_srvs/srv/Empty
```

Close the tool arm:

```
$ ros2 service call /close_arm std_srvs/srv/Empty
```

Open the mast (camera arm)

```
$ ros2 service call /mast_open std_srvs/srv/Empty
```

Close the mast (camera arm)

```
$ ros2 service call /mast_close std_srvs/srv/Empty
```
