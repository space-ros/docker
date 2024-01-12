# RTEMS
This folder contains a Dockerfile that build RTEMS from source following the [official doc](https://docs.rtems.org/branches/master/user/overview/index.html).
The target platform for the build is leon3.
This will not run RTEMS, just perform the build and set up tools.

## Examples
There three examples in this demo: example, hello and tinxyxml2.
hello is ported from `rtems-cmake`, uses waf to cross-compile.
example is an hello world example build using `sparc-rtems5-gcc` tool.
tinyxml2 is modified to work with RTEMS build using `sparc-rtems5-gcc` tool..

## Compile
```bash
cd /root/tinxyxml2
./doit
```

## Save build artefacts
After compiling the binaries for the target, save the binaries and test payload for later use in renode.

```bash
docker cp containerId:/root/tinyxml2/tinyxml2.o <path to spaceros_ws>/docker/renode_rcc/build/

docker cp containerId:/root/tinyxml2/xmltest.exe <path to spaceros_ws>/docker/renode_rcc/build/
```
