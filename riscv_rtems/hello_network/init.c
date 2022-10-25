
/*
 * Simple RTEMS configuration
 */

// not sure if this is necessary, since we're not starting any of the
// BSD-provided services like PF, telnet, etc.
#define RTEMS_BSD_CONFIG_INIT
#include <machine/rtems-bsd-config.h>

#define CONFIGURE_USE_IMFS_AS_BASE_FILESYSTEM
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_UNLIMITED_OBJECTS
#define CONFIGURE_UNIFIED_WORK_AREAS

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_MAXIMUM_USER_EXTENSIONS 1

// this defaults to 3 (stdout, stdin, stderr) and we need more...
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 32

#define CONFIGURE_INIT

#include <rtems/confdefs.h>

