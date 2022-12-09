#!/bin/bash
colcon build \
	--executor sequential \
	--merge-install \
	--cmake-force-configure \
	--event-handlers console_direct+ \
	--cmake-args \
		-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
		-DCMAKE_TOOLCHAIN_FILE=/root/toolchain.cmake \
		-DBUILD_TESTING=OFF \
		-DBUILD_SHARED_LIBS=OFF \

		#-DSECURITY=OFF \
		#-DENABLE_SSL=NO \
		#-DENABLE_SECURITY=NO
