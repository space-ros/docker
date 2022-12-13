#include <assert.h>
#include <sysexits.h>
#include <rtems.h>
#include <rtems/bsd/bsd.h>
#include <rtems/bsd/iface.h>
#include <machine/rtems-bsd-commands.h>

void network_init()
{
  rtems_bsd_setlogpriority("debug");

  rtems_status_code sc;
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

}
