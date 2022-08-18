# MoveIt2 Docker Image

The MoveIt2 Docker image uses the Space ROS docker image (*openrobotics/spaceros:latest*) as its base image. The MoveIt2 Dockerfile installs all of the prerequisite system dependencies to build MoveIt2 and then pulls and builds the latest MoveIt2 source code.

## Building the MoveIt2 Image

To build the docker image, run:

```
$ ./build.sh
```

The build process will take about 30 minutes, depending on the host computer.

## Running the MoveIt2 Docker Image in a Container

After building the image, you can see the newly-built image by running:

```
$ docker image list
```

The output will look something like this:

```
REPOSITORY              TAG                        IMAGE ID       CREATED        SIZE
openrobotics/moveit2    latest                     6edb2edc9643   10 hours ago   15.5GB
openrobotics/spaceros   latest                     629b13cf7b74   12 hours ago   7.8GB
nvidia/cudagl           11.4.1-devel-ubuntu20.04   336416dfcbba   1 week ago     5.35GB
```

The new image is named **openrobotics/moveit2:latest**.

There is a run.sh script provided for convenience that will run the spaceros image in a container.

```
$ ./run.sh
```

Upon startup, the container automatically runs the moveit2_entrypoint.sh script, which sources the MoveIt2 and Space ROS environment files. You'll now be running inside the container and should see a prompt similar to this:

```
root@8e73b41a4e16:/root/src/moveit2_ws# 
```
