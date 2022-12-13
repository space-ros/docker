// not sure if this is necessary, since we're not starting any of the
// BSD-provided services like PF, telnet, etc.
#define RTEMS_BSD_CONFIG_BSP_CONFIG
#define RTEMS_BSD_CONFIG_INIT

#include <machine/rtems-bsd-config.h>

#define CONFIGURE_USE_IMFS_AS_BASE_FILESYSTEM
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_STUB_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_ZERO_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_LIBBLOCK


#define CONFIGURE_UNLIMITED_OBJECTS
#define CONFIGURE_UNIFIED_WORK_AREAS

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_MAXIMUM_USER_EXTENSIONS 1

// this defaults to 3 (stdout, stdin, stderr) and we need a lot more...
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 128

#define CONFIGURE_INIT_TASK_STACK_SIZE (32 * 1024)

#define CONFIGURE_STACK_CHECKER_ENABLED

// currently we're only using 1, but perhaps another will be used sometime
#define CONFIGURE_MAXIMUM_PERIODS 2
#define CONFIGURE_MICROSECONDS_PER_TICK 1000

#define CONFIGURE_INIT

#include <rtems/confdefs.h>
