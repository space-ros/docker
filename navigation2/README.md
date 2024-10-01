# Navigation2 Docker Image

The Navigation2 Docker image uses the Space ROS docker image (*osrf/space-ros:latest*) as its base image.
The Navigation2 Dockerfile installs all of the prerequisite system dependencies to build Navigation2
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
REPOSITORY              TAG                        IMAGE ID       CREATED        SIZE
osrf/space_nav2         latest                     6edb2edc9643   10 hours ago   15.5GB
openrobotics/spaceros   latest                     629b13cf7b74   12 hours ago   7.8GB
nvidia/cudagl           11.4.1-devel-ubuntu20.04   336416dfcbba   1 week ago     5.35GB
```

The new image is named **osrf/space_nav2:latest**.

There is a run.sh script provided for convenience that will run the spaceros image in a container.

```
./run.sh
```

Upon startup, the container automatically runs the entrypoint.sh script, which sources the MoveIt2 and Space ROS environment files.
You'll now be running inside the container and should see a prompt similar to this:

```
root@ip-your-ip-address:/home/spaceros-user/nav2_ws#
```

## Running Navigation2 Demo

To run the latest demo, see the README in the nav2_demos folder
