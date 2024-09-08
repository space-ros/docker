# Integrating Open 3D Engine with Space ROS for an Accurate Physics Simulation of the Curiosity Rover: A Case Study

This docker image is part of a submission to the NASA Space ROS Sim Summer Spring Challenge 2024.
Related Issue: [#181](https://github.com/space-ros/docker/issues/181)

Author: Azmyin Md. Kamal, 
Ph.D. student, iCORE Lab, MIE
Louisiana State University,
Baton Rounge, LA-70803

Date: 09/08/2024


**TODO** a preview image once Bulding 336 asset is complete.

## Description

This README.md contains the build and usage instructions for **o3de_curiosity_demo**, a docker image that demonstrates the Curiosity rover traversing through a new test enviornment modelled after NASA JPL's Mars Yard testing ground. It is meant to showcase the integration of the Open 3D Engine with Space ROS via the [ROS 2 Gem](). All textures used are CC0, all models built are either from CC0 sources or made by me which, I hereby declare as also CC0. Open 3D Engine, ROS 2 Gem are all Apache 2.0 licensed. Space ROS and all other releated dependecies are also Apache 2.0 licensed. Please look into the attached url for seeing the demo in action.

**TODO** link to the 5 min video that will be submitted as part of the competition.


The Dockerfile installs all of the prerequisite system dependencies for building the ros2 workspace, installs the Open 3D Engine, registers the engine and then downloads, and registers in order, the ```RobotSim``` O3DE project, the ```NasaCuriosityRover``` Gem and the ```MarsYard``` Gem. Finally, the docker file will build the entire project (takes 5 - 10 mins).

## Prerequisits

* Ensure Docker and then Earthly are installed correctly.

* Git clone this fork of ```space-ros/docker``` and change directory

```bash
cd ~
git clone --branch o3de_curiosity_docker --single-branch https://github.com/Mechazo11/space-ros-docker.git
cd space-ros-docker
```

* This docker image reqiures **space-ros** base image ```osrf/spaceros:latest```. Pull the latest build from docker hub

```bash
docker pull osrf/space-ros
```

* Buld moveit2 docker image

uses the moveit2 docker image (*openrobotics/moveit2:latest*) as its base image.
Build instructions for that image can be found in [this README](../moveit2/README.md).


along with the demos source code, then builds the Space ROS Space Robots Demo.

This is for Curiosity Mars rover and Canadarm demos.

