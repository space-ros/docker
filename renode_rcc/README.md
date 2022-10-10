# renode-RCC
Building [RTEMS Cross Compilation System (RCC)](https://www.gaisler.com/index.php/products/operating-systems/rtems), has a different directory structure compare to building RTEMS from source (might not be true). This will run RTEMS on leon3 in a renode simulation.

## Usage
To run the simulator in docker container
```
renode
```

> If you face GTK protocol error then exit the container, run `xhost + local:` and restart the conatiner to allow other users (including root) run programs in the current session.


In the renode window, run
```
start
s @renode-rtems-leon3/leon3_rtems.resc
```
