#include <assert.h>
#include <rtems.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sysexits.h>
#include <rtems/bsd/bsd.h>
#include <rtems/bsd/iface.h>
#include <rtems/printer.h>
#include <rtems/stackchk.h>
#include <rtems/bspIo.h>
#include <machine/rtems-bsd-commands.h>

static rtems_id task_ids[1];
rtems_task PubTask(rtems_task_argument ignored);

static void my_on_exit(int exit_code, void * /*arg*/)
{
  //(void)arg; // suppress warning
  rtems_printer printer;
  rtems_print_printer_printf(&printer);
  rtems_stack_checker_report_usage_with_plugin(&printer);
  rtems_cpu_usage_report();
}

static void
default_wait_for_link_up( const char *name )
{
  size_t seconds = 0;
  while ( true ) {
    bool link_active = false;
    assert(rtems_bsd_iface_link_state( name, &link_active ) == 0);
    if (link_active) {
      return;
    }
    sleep( 1 );
    ++seconds;
    if (seconds > 10) {
      printf("error: %s: no active link\n", name);
      assert(seconds < 10);
    }
  }
}

rtems_task Init(rtems_task_argument ignored)
{
  rtems_bsd_setlogpriority("debug");
  printf("\nInit() start\n\n");
  on_exit(my_on_exit, NULL);

  rtems_status_code sc;

  // set ourselves as minimum priority
  rtems_task_priority priority = RTEMS_MAXIMUM_PRIORITY - 1;
  sc = rtems_task_set_priority(RTEMS_SELF, priority, &priority);
  assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_bsd_initialize();
  assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_task_wake_after(2); // not sure of the units... maybe 10ms ticks?
  rtems_bsd_ifconfig_lo0();

  int exit_code = 0;

  char *iface_name = "cgem3";
  char *ifcfg_set[] = {
    "ifconfig",
    iface_name,
    "inet",
    "10.0.42.100",
    "netmask",
    "255.255.255.0",
    NULL
  };
  exit_code = rtems_bsd_command_ifconfig(RTEMS_BSD_ARGC(ifcfg_set), ifcfg_set);
  assert(exit_code == EX_OK);
  /*
  sc = rtems_task_wake_after(100); // not sure of the units... maybe 10ms ticks?
  char *route_print[] = {
    "netstat",
    "-r",
    NULL
  };
  exit_code = rtems_bsd_command_netstat(RTEMS_BSD_ARGC(route_print), route_print);
  if (exit_code != EX_OK)
  {
    printk("print routes exit code: %d\n", exit_code);
    exit(1);
  }
  */

  char *route_default[] = {
    "route",
    "add",
    "default",
    "10.0.42.1",
    NULL
  };
  exit_code = rtems_bsd_command_route(RTEMS_BSD_ARGC(route_default), route_default);
  if (exit_code != EX_OK)
  {
    printk("set routes exit code: %d\n", exit_code);
    exit(1);
  }

  // start the rate-monotonic (periodic) task
  rtems_status_code status;
  status = rtems_task_create(
    rtems_build_name('P', 'U', 'B', '1'),
    2, // task priority
    32 * 1024, // stack size
    RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES,
    &task_ids[0]
  );
  if (status != RTEMS_SUCCESSFUL)
  {
    printk("unexpected task_create status code: %d\n", status);
    exit(1);
  }

  status = rtems_task_start(task_ids[0], PubTask, 0);
  if (status != RTEMS_SUCCESSFUL)
  {
    printk("unexpected task_start status code: %d\n", status);
    exit(1);
  }

  //struct timeval tv;
  //printf("publishing %d / %d\n", i, num_pub);

  rtems_task_exit();
}
