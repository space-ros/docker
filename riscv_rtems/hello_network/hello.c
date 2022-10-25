
/*
 * Hello world example
 */
#include <assert.h>
#include <rtems.h>
#include <stdlib.h>
#include <stdio.h>
#include <sysexits.h>
#include <rtems/bsd/bsd.h>
//#include <rtems/bsd/modules.h>
#include <rtems/printer.h>
#include <rtems/stackchk.h>
#include <rtems/bspIo.h>
#include <machine/rtems-bsd-commands.h>

static void my_on_exit(int exit_code, void *arg)
{
  rtems_printer printer;
  (void)arg; // suppress warning
  rtems_print_printer_printf(&printer);
  rtems_stack_checker_report_usage_with_plugin(&printer);
}

rtems_task Init(
  rtems_task_argument ignored
)
{
  rtems_bsd_setlogpriority("debug");
  printk("\nInit() start\n\n");
  on_exit(my_on_exit, NULL);

  rtems_status_code sc;

  sc = rtems_bsd_initialize();
  assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_task_wake_after(2); // not sure of the units... maybe 10ms ticks?
  rtems_bsd_ifconfig_lo0();
  // rtems_bsd_run_rc_conf_script(
  //   "my_tap_script",
  //   "hostname=\"my_host\"\n"
  //   "")

  //char *ifname = NET_CFG_INTERFACE_0;
  char *ifcfg[] = {
    "ifconfig",
    NULL
  };
  int exit_code = rtems_bsd_command_ifconfig(RTEMS_BSD_ARGC(ifcfg), ifcfg);
  if (exit_code != EX_OK)
  {
    printk("ifconfig exit code: %d\n", exit_code);
    exit(1);
  }
  //assert(exit_code == EX_OK);

  /*
  exit_code = rtems_bsd_ifconfig_lo0();
  assert(exit_code == EX_OK);
  */

  /*
  int exit_code;
  char *ping[] = {
    "ping",
    "-c",
    "1",
    "127.0.0.1",
    NULL
  };

  exit_code = rtems_bsd_command_ping(RTEMS_BSD_ARGC(ping), ping);
  assert(exit_code == EXIT_SUCCESS);
  */

  exit(0);
}
