# Navigation2 Docker Image

The Navigation2 Docker image uses the Space ROS docker image (*osrf/space-ros:latest*) as its base image.
The Navigation2 Dockerfile installs all of the prerequisite system dependencies to build Navigation2.
All the required nav2 packages are cloned and built from sources using the `navigation2.repos` file. The `nav2_rviz_plugins` and `nav2_bringup` packages together with their dependencies (Rviz, Gazebo, etc) are excluded to reduce package dependencies. The image also contains a custom `space_nav2_bringup` package that can be used as a starting point for creating mission-specific nav2 launch files.

## Building the Navigation2 Image

To build the docker image, run:

```
./build.sh
```

The build process will take about 30 minutes, depending on the host computer.

By default, this will build on top of the latest released version of the Space ROS base image (typically `osrf/space-ros:latest`).
If building locally, the underlying base image can be set in the [build script](./build.sh), or through the environment with:

```bash
# Use a locally built image as the base
SPACE_ROS_IMAGE="space-ros:main" ./build.sh
```

## Running the Navigation2 Docker Image in a Container

After building the image, you can see the newly-built image by running:

```
docker image list
```

The output will look something like this:

```
REPOSITORY                       TAG       IMAGE ID       CREATED          SIZE
osrf/space_nav2                  latest    7016292fba1c   32 minutes ago   3.65GB
osrf/space-ros                   latest    cf10cd2cb82c   4 days ago       1.05GB
```

The new image is named **osrf/space_nav2:latest**.

There is a run.sh script provided for convenience that will run the spaceros image in a container.

```
./run.sh
```

Upon startup, the container automatically runs the entrypoint.sh script, which sources the Nav2 and Space ROS environment files.
You'll now be running inside the container and should see a prompt similar to this:

```
spaceros-user@ip-your-ip-address:/home/spaceros-user/nav2_ws#
```

## Running Navigation2 Demo

To run the latest demo, see the README in the [nav2_demo](../nav2_demo/README.md) folder.

## Updating navigation2 packages

The `navigation2.repos` file available in this repository provides a list of repos that are required to build the nav2 stack. ROS packages already included in the SpaceROS base image are omited from this file.

To update navigation2 packages used to build the space nav2 image, run the `docker_update_nav2_repos.sh` script:

```
./docker_update_nav2_repos.sh
```

This will update the `navigation2.repos` file with the newest versions of nav2 packages. The script resolves missing packages in base spaceros image and creates a list with their latest versions. The `navigation2.repos` file is then copied during build to the docker image with space nav2, and packages specified in the file are cloned and installed.

By default, this update nav2 repositories using the latest released version of the Space ROS base image (typically `osrf/space-ros:latest`).
If building locally, the underlying base image can be set in the through the environment with:

```
SPACE_ROS_IMAGE=osrf/space-ros:main ./docker_update_nav2_repos.sh
```

To generate a list of required packages for any other ROS workspace (e.g. your custom workspace built on top of SpaceROS that already has some nav2 dependencies installed), consider using the `update_nav2_repos.sh` as your starting point. You may also need to clone `generate-repos.sh` script from the main spaceros repository.
