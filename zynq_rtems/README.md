# Space-ROS Zynq RTEMS Demo

(Please note, this demonstration is a work in progress)

The goal is to create an hard real-time embedded application that runs on a Xilinx Zynq SoC FPGA which will communicate with Space-ROS.

This demonstration will be on [QEMU](https://www.qemu.org), the open-source CPU and system emulator.
This is for several reasons:
 * Development cycles in QEMU are much faster and less painful than hardware.
 * Given the state of the semiconductor industry at time of writing (late 2022), it is nearly impossible to obtain Zynq development boards.
 * QEMU-based work requires no upfront hardware costs or purchasing delays

There are a number of reasonable choices available when selecting a real-time OS (RTOS).
We selected [RTEMS](https://www.rtems.org/) because it is fully open-source and has a long history of successful deployments in space applications.

# Build

To simplify collecting and compiling dependencies on a wide range of systems, we have created a Docker container that contains everything.
You will need to install Docker on your system first, for example, using `sudo apt install docker.io`
Then, run this [script](./build_dependencies.sh):

```bash
cd /path/to/zynq_rtems
./build_dependencies.sh
```

This will build the [zynq_rtems Dockerfile](./Dockerfile), which builds QEMU, a cross-compile toolchain for the ARMv8 processor inside the Zynq SoC, and RTEMS from source in the container.
This will typically take at least 10 minutes, and can take much longer if either your network connection or compute resources is limited.

Next, we will use this "container full of dependencies" to compile a sample application.

```bash
cd /path/to/zynq_rtems
./compile_demos.sh
```

The demo application and its cross-compiled RTEMS kernel will be accessible both "inside" and "outside" the container for ease of source control, editing cycles, and debugging.
The build products will land in `zynq_rtems/hello_zenoh/build`.

# Run

The emulated system that will run inside QEMU needs to have a way to talk to a virtual network segment on the host machine.
We'll create a TAP device for this.
The following script will set this up, creating a virtual `10.0.42.x` subnet for a device named `tap0`:

```bash
./start_network_tap.sh
```

We will need three terminals for this demo:
 * Zenoh router
 * Zenoh subscriber
 * Zenoh-Pico publisher (in RTEMS in QEMU)

First, we will start a Zenoh router:

```bash
cd /path/to/zynq_rtems
cd hello_zenoh
./run_zenoh_router
```
This will print a bunch of startup information and then continue running silently, waiting for inbound Zenoh traffic.
Leave this terminal running.

In the second terminal, we'll run the Zenoh subscriber example:

```bash
cd /path/to/zynq_rtems
cd hello_zenoh
./run_zenoh_subscriber
```

In the third terminal, we will run the RTEMS-based application, which will communicate with the Zenoh router and thence to the Zenoh subscriber.
The following script will run QEMU inside the container, with a volume-mount of the `hello_zenoh` demo application so that the build products from the previous step are made available to the QEMU that was built inside the container.

```bash
cd /path/to/zynq_rtems
cd hello_zenoh
./run_rtems.sh
```

The terminal should print a bunch of information about the various emulated Zynq network interfaces and their routing information.
After that, it should contact the `zenohd` instance running in the other terminal.
It should print something like this:

```
Opening zenoh session...
Zenoh session opened.
Own ID: 0000000000000000F45E7E462568C23B
Routers IDs:
 B2FE444C3B454E27BCB11DF83120D927
Peers IDs:
Stopping read and lease tasks...
sending a few messages...
publishing: Hello, world! 0
publishing: Hello, world! 1
publishing: Hello, world! 2
publishing: Hello, world! 3
publishing: Hello, world! 4
publishing: Hello, world! 5
publishing: Hello, world! 6
publishing: Hello, world! 7
publishing: Hello, world! 8
publishing: Hello, world! 9
Closing zenoh session...
Done. Goodbye.
```

The second terminal, running a Zenoh example subscriber, should print something like this:
```
Declaring Subscriber on 'example'...
[2022-12-06T21:41:11Z DEBUG zenoh::net::routing::resource] Register resource example
[2022-12-06T21:41:11Z DEBUG zenoh::net::routing::pubsub] Register client subscription
[2022-12-06T21:41:11Z DEBUG zenoh::net::routing::pubsub] Register client subscription example
[2022-12-06T21:41:11Z DEBUG zenoh::net::routing::pubsub] Register subscription example for Face{0, 5F6D54C4366D42EDB367F17A5A2CACCD}
Enter 'q' to quit...
>> [Subscriber] Received PUT ('example': 'Hello, world! 0')
>> [Subscriber] Received PUT ('example': 'Hello, world! 1')
>> [Subscriber] Received PUT ('example': 'Hello, world! 2')
>> [Subscriber] Received PUT ('example': 'Hello, world! 3')
>> [Subscriber] Received PUT ('example': 'Hello, world! 4')
>> [Subscriber] Received PUT ('example': 'Hello, world! 5')
>> [Subscriber] Received PUT ('example': 'Hello, world! 6')
>> [Subscriber] Received PUT ('example': 'Hello, world! 7')
>> [Subscriber] Received PUT ('example': 'Hello, world! 8')
>> [Subscriber] Received PUT ('example': 'Hello, world! 9')
```

After that output, the RTEMS shutdown will display the various RTEMS threads running and their memory usage.

This showed that the Zenoh Pico client running in RTEMS successfully reached the Zenoh router running natively on the host.
Success!
This is a good thing.

# Clean up

If you would like, you can now remove the network tap device that we created in the previous step:

```bash
zynq_rtems/stop_network_tap.sh
```
