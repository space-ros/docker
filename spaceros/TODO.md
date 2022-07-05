This is a work-in-progress branch for using [earthly](https://earthly.dev) to manage Space ROS builds.

I've currently converted the spaceros docker file build into an Earthfile (ugh, Filefiles) which has the build process broken into some chosen steps.

A goal is to allow the fetching of sources to occur in an automated fashion while still affording local modifications and using the same build procedures for both.

To maintain that decoupling, initial setup requires you to run

```
earthly +sources
```

This will fetch the spaceros `ros2.repos` file and perform a vcs checkout mirrored to a local `spaceros_ws/src` directory.

Further earthly targets will use the _local_ directory rather than refreshing the sources, allowing you to run builds with modified sources without first pushing and generating a repos file.

To run the build use the `+build` target, and to create the spaceros docker image, containing only the install directory, which is not sufficient to run tests, use the `+image` target.

```
earthly +image
```
