# renode-RCC
Building [RTEMS Cross Compilation System (RCC)](https://www.gaisler.com/index.php/products/operating-systems/rtems), has a different directory structure compare to building RTEMS from source (might not be true). This will run RTEMS on leon3 in a renode simulation.

## Usage
To run the simulator in docker container
```
renode
```

In the renode window, run
```
start
s @renode-rtems-leon3/leon3_rtems.resc
```
