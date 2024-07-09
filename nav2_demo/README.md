# This folder contains the necessary files for running Nav2 on the Mars Rover demo

# This demo is a preliminary release, please file any issues found

## Demo description
In the demo, a mars rover is spawned in gazebo simulation environment, and space nav2 stack built on top of SpaceROS is used for autonomous navigation.
ROS2 container with Rviz2 is used to set the goal and visualize the robot.

## First build this container using the build script

```
./build.sh
```

There will now be two docker images called "osrf/space_nav2_demo:latest"  and "osrf/rviz_space_nav2:latest".

# Run the Mars Rover demo with Nav2 and SLAM toolbox

## Terminal 1 - launch the mars_rover demo

Follow instructions on [../space_robots/README.md.](../space_robots/README.md)
This will launch the Gazebo simulation with mars rover and required controllers.

## Terminal 2 - launch Nav2

Start the space_nav2 container and launch the navigation2 nodes:

```
./run_space_nav2.sh
```
```
ros2 launch space_nav2_bringup navigation_launch.py use_sim_time:=True params_file:=nav2_params.yaml
```

## Terminal 3 - launch localization with map

```
docker exec -it osrf_space_nav2_demo bash
```
```
source install/setup.bash
ros2 launch space_nav2_bringup localization_launch.py use_sim_time:=True map:=mars_map.yaml params_file:=nav2_params.yaml
```

## Terminal 4 - launch Rviz

The `space_nav2` is a lightweight image built on top of SpaceROS base image and does not include Rviz2. We can launch Rviz2 using a separate image, built on top of ROS2:

```
./run_rviz2.sh
```
```
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true
```

## Localize the robot and send a Nav2 goal

In the Rviz GUI, localize robot in Rviz using 2D Pose estimate tool, with an arrow pointing up at the center of the grid.
Wait for Nav2 to initialize / costmaps to appear.
When the costmap appears, send a goal using the "Nav2 goal" tool in Rviz
