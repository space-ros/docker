#!/bin/bash
set -euo pipefail

wget --progress=dot:giga https://www.gaisler.com/anonftp/rcc/bin/linux/sparc-rtems-5-gcc-10.2.0-1.3.0-linux.txz
tar -Jxf sparc-rtems-5-gcc-10.2.0-1.3.0-linux.txz
pushd rcc-1.3.0-gcc/src/samples/
# Add rtems-shell application to LEON3 target
# as by default it is included only for LEON2
sed -i.bkp '/LEON3_ALL= \\/a rtems-shell \\' Makefile
make -j`nproc` leon3
popd
