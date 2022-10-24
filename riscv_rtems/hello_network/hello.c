
/*
 * Hello world example
 */
#include <rtems.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <rtems/bsd/bsd.h>
//#include <machine/rtems-bsd-commands.h>
//#include <rtems/bsd/test/network-config.h>

rtems_task Init(
  rtems_task_argument ignored
)
{
  printk( "\nnetwork init start\n" );

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

  rtems_status_code sc;

  sc = rtems_bsd_initialize();
  assert(sc == RTEMS_SUCCESSFUL);

  /*
  exit_code = rtems_bsd_ifconfig_lo0();
  assert(exit_code == EX_OK);
  */

  exit( 0 );
}
