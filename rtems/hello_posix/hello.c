#include <rtems.h>
#include <stdlib.h>
#include <stdio.h>
#include <rtems/stackchk.h>

void *POSIX_Init()
{
  printf("\nHello, POSIX world! %d\n", 42);
  rtems_stack_checker_report_usage();
  exit(0);
}
