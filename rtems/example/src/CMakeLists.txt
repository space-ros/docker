cmake_minimum_required(VERSION 3.2)

set(RTEMS_CONFIG_DIR
	"/root/rtems-cmake"
	 CACHE FILEPATH "Directory containing the RTEMS *.cmake files"
)

set(RTEMS_PREFIX
	"/root/development/rtems/5"
	CACHE FILEPATH "Prefix path for RTEMS"
)

set(RTEMS_BSP
	"sparc/leon3"
	CACHE FILEPATH "Board support package name"
)


include(${RTEMS_CONFIG_DIR}/RTEMSPreProjectConfig.cmake)
rtems_pre_project_config(${RTEMS_PREFIX} ${RTEMS_BSP})

set(CMAKE_TOOLCHAIN_FILE ${RTEMS_CONFIG_DIR}/RTEMSToolchain.cmake)

project(Example)
add_executable(Example main.cpp)

target_include_directories(Example PUBLIC /root/development/rtems/5/sparc-rtems5/leon3/lib/include)
