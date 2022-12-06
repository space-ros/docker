# Space-ROS Zynq RTEMS Demo

(Please note, this demonstration is a work in progress)

The goal is to create an hard real-time embedded application that runs on a
Xilinx Zynq SoC FPGA which will communicate with Space-ROS.

This demonstration will be on [QEMU](https://www.qemu.org), the open-source CPU
and system emulator. This is for several reasons. First, development cycles in
QEMU are much faster and less painful than using real hardware. Second, given
the state of the semiconductor industry at time of writing (late 2022), it is
nearly impossible to obtain Zynq development boards. Third, a QEMU-based
demonstration requires no upfront costs or delays.

There are a number of reasonable choices available when selecting a real-time
OS (RTOS).  We selected [RTEMS](https://www.rtems.org/) because it is fully
open-source and has a long history of successful deployments in space
applications.

# Build

To simplify collecting and compiling dependencies on a wide range of systems,
we have created a Docker container that contains everything. You will need
to install Docker on your system first. Then, run this [script](https://github.com/space-ros/docker/blob/zynq_rtems/zynq_rtems/build_dependencies.sh):

```
cd /path/to/zynq_rtems
./build_dependencies.sh
```

This will use the [zynq_rtems Dockerfile](https://github.com/space-ros/docker/blob/zynq_rtems/zynq_rtems/Dockerfile), which builds QEMU, a cross-compile toolchain for the ARMv8 processor inside the Zynq SoC, and RTEMS from source in the container. This will typically take at least 10 minutes, and can take much longer if either your network connection or compute resources is limited.

Next, we will use this "container full of dependencies" to compile a sample "Hello, World" application which will just ping the host from within the container:

```
cd /path/to/zynq_rtems
./compile_demos.sh
```

The demo application and its cross-compiled RTEMS kernel will live "outside" the container for ease of source control, editing cycles, and debugging. The build products will land in `zynq_rtems/hello_network/build`.

# Run

The emulated system that will run inside QEMU needs to have a way to talk to a virtual network segment on the host machine. We'll create a TAP device for this. The following script will set this up, creating a virtual `10.0.42.x` subnet for a device named `tap0`:
```
zynq_rtems/start_network_tap.sh
```

Now we can run the RTEMS-based application built in the previous step. The following script will run QEMU inside the container, with a volume-mount of the `hello_network` demo application so that the build products from the previous step are made available to the QEMU that was built inside the container.
```
cd /path/to/zynq_rtems
cd hello_network
./run_qemu.sh
```

You should see the following output, which shows that an emulated network interface on the emulated Zynq SoC is able to `ping` the virtual network segment provided by the host machine's Linux kernel:
```
PING 10.0.42.1 (10.0.42.1): 56 data bytes
64 bytes from 10.0.42.1: icmp_seq=0 ttl=64 time=5.846 ms
64 bytes from 10.0.42.1: icmp_seq=1 ttl=64 time=0.598 ms
64 bytes from 10.0.42.1: icmp_seq=2 ttl=64 time=0.484 ms

--- 10.0.42.1 ping statistics ---
3 packets transmitted, 3 packets received, 0.0% packet loss
round-trip min/avg/max/stddev = 0.484/2.309/5.846/2.501 ms
```

After that output, the RTEMS shutdown will display the various RTEMS threads running and their memory usage.

# Clean up

If you would like, you can now remove the network tap device that we created in the previous step:
```
zynq_rtems/stop_network_tap.sh
```
