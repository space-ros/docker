#if 0
#include <assert.h>
#include <rtems.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <rtems/cpuuse.h>
#include <rtems/printer.h>
#include <rtems/stackchk.h>
#include <rtems/bspIo.h>

// forward declarations
void network_init();
void pub_init();

static void my_on_exit(int exit_code, void * /*arg*/)
{
  rtems_printer printer;
  rtems_print_printer_printf(&printer);
  rtems_stack_checker_report_usage_with_plugin(&printer);
  rtems_cpu_usage_report();
}

rtems_task Init(rtems_task_argument ignored)
{
  printf("\nInit() start\n\n");
  on_exit(my_on_exit, NULL);

  rtems_status_code sc;

  // set ourselves as minimum priority
  rtems_task_priority priority = RTEMS_MAXIMUM_PRIORITY - 1;
  sc = rtems_task_set_priority(RTEMS_SELF, priority, &priority);
  assert(sc == RTEMS_SUCCESSFUL);

  network_init();
  pub_init();

  rtems_task_exit();
}
#endif
