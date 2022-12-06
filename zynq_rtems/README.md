# Space-ROS Zynq RTEMS Demo

(Please note, this demonstration is a work in progress)

The goal is to create an hard real-time embedded application that runs on a Xilinx Zynq SoC FPGA which will communicate with Space-ROS.

This demonstration will be on [QEMU](https://www.qemu.org), the open-source CPU and system emulator. This is for several reasons:
 * Development cycles in QEMU are much faster and less painful than hardware.
 * Given the state of the semiconductor industry at time of writing (late 2022), it is nearly impossible to obtain Zynq development boards.
 * QEMU-based work requires no upfront hardware costs or purchasing delays

There are a number of reasonable choices available when selecting a real-time OS (RTOS).
We selected [RTEMS](https://www.rtems.org/) because it is fully open-source and has a long history of successful deployments in space applications.

# Build

To simplify collecting and compiling dependencies on a wide range of systems, we have created a Docker container that contains everything.
You will need to install Docker on your system first, for example, using `sudo apt install docker.io`
Then, run this [script](https://github.com/space-ros/docker/blob/zynq_rtems/zynq_rtems/build_dependencies.sh):

```
cd /path/to/zynq_rtems
./build_dependencies.sh
```

This will build the [zynq_rtems Dockerfile](https://github.com/space-ros/docker/blob/zynq_rtems/zynq_rtems/Dockerfile), which builds QEMU, a cross-compile toolchain for the ARMv8 processor inside the Zynq SoC, and RTEMS from source in the container.
This will typically take at least 10 minutes, and can take much longer if either your network connection or compute resources is limited.

Next, we will use this "container full of dependencies" to compile a sample application.

```
cd /path/to/zynq_rtems
./compile_demos.sh
```

The demo application and its cross-compiled RTEMS kernel will be accessible both "inside" and "outside" the container for ease of source control, editing cycles, and debugging.
The build products will land in `zynq_rtems/hello_network/build`.

# Run

The emulated system that will run inside QEMU needs to have a way to talk to a virtual network segment on the host machine.
We'll create a TAP device for this.
The following script will set this up, creating a virtual `10.0.42.x` subnet for a device named `tap0`:
```
zynq_rtems/start_network_tap.sh
```

Next, we will start a Zenoh router, which was built inside the container:
```
zynq_rtems/hello_zenoh/run_zenoh_router
```
This will print a bunch of startup information and then continue running silently, waiting for inbound Zenoh traffic. Leave this terminal running.

Now, start a new terminal.
In this new terminal, we will run the RTEMS-based application, which will communicate with the Zenoh router.
The following script will run QEMU inside the container, with a volume-mount of the `hello_zenoh` demo application so that the build products from the previous step are made available to the QEMU that was built inside the container.
```
cd /path/to/zynq_rtems
cd hello_network
./run_rtems.sh
```

The terminal should print a bunch of information about the various emulated Zynq network interfaces and their routing information.
After that, it should contact the `zenohd` instance running in the other terminal. It should print something like this:
```
Opening zenoh session...
Zenoh session opened.
Own ID: 0000000000000000F45E7E462568C23B
Routers IDs:
 B2FE444C3B454E27BCB11DF83120D927
Peers IDs:
Stopping read and lease tasks...
Closing zenoh session...
Done. Goodbye.
```

After that output, the RTEMS shutdown will display the various RTEMS threads running and their memory usage.

This showed that the Zenoh Pico client running in RTEMS successfully reached the Zenoh router running natively on the host.
Success!
This is a good thing.

# Clean up

If you would like, you can now remove the network tap device that we created in the previous step:
```
zynq_rtems/stop_network_tap.sh
```
