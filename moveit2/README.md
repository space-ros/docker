# MoveIt2 Docker Image

The MoveIt2 Docker image uses the Space ROS docker image (*openrobotics/spaceros:latest*) as its base image.
The MoveIt2 Dockerfile installs all of the prerequisite system dependencies to build MoveIt2 (and Moveit2 tutorials) and then pulls and builds the latest MoveIt2 and Moveit2 tutorials source code.

## Building the MoveIt2 Image

To build the docker image, run:

```bash
./build.sh
```

By default, this will build on top of the latest released version of the Space ROS base image (typically `osrf/space-ros:latest`).
If building locally, the underlying base image can be set in the [build script](./build.sh), or through the environment with:

```bash
# Use a locally built image as the base
SPACE_ROS_IMAGE="osrf/space-ros:latest" ./build.sh
```

Similarly, the tag for the resulting Docker image can be customized using the `MOVEIT2_TAG` environment variable.
For example, to build an image based on the `osrf/space-ros:jazzy-2025.04.0` image and tag it with the same release label use:

```bash
# Use a locally built image as the base
SPACE_ROS_IMAGE="osrf/space-ros:jazzy-2025.04.0" MOVEIT2_TAG="jazzy-2025.04.0" ./build.sh
```

The build process will take about 45 minutes, depending on the host computer.

## Running the MoveIt2 Docker Image in a Container

After building the image, you can see the newly-built image by running:

```bash
docker image list
```

The output will look something like this:

```bash
REPOSITORY             TAG       IMAGE ID       CREATED             SIZE
openrobotics/moveit2   latest    bd7342baeff2   4 hours ago         5.49GB
osrf/space-ros         latest    18a3c6709c93   6 hours ago         1.37GB
```

The new image is named **openrobotics/moveit2:latest**.

There is a run.sh script provided for convenience that will run the spaceros image in a container.

```bash
./run.sh
```

Upon startup, the container automatically runs the entrypoint.sh script, which sources the MoveIt2 and Space ROS environment files.
You'll now be running inside the container and should see a prompt similar to this:

```
spaceros-user@8e73b41a4e16:~/moveit2#
```

## Running the Space ROS Space Robots Demos

Once you have tested that MoveIt2 works, you are ready to run some of the other [Space ROS space robot demos](../space_robots/README.md).
