#include <rtems.h>
#include <stdio.h>
#include <stdlib.h>
#include "diagnostic_msgs/msg/diagnostic_array.h"
#include "diagnostic_msgs/msg/diagnostic_status.h"
#include "diagnostic_msgs/msg/key_value.h"
#include "rosidl_runtime_c/string_functions.h"

static rtems_id daq_task_id;
rtems_task DaqTask(rtems_task_argument ignored);

void daq_init()
{
  // start the rate-monotonic (periodic) task
  rtems_status_code status;
  status = rtems_task_create(
    rtems_build_name('D', 'A', 'Q', '1'),
    200, // task priority
    32 * 1024, // stack size
    //RTEMS_DEFAULT_MODES,
    RTEMS_PREEMPT | RTEMS_TIMESLICE | RTEMS_ASR | RTEMS_INTERRUPT_LEVEL(0),
    RTEMS_DEFAULT_ATTRIBUTES,
    &daq_task_id
  );
  if (status != RTEMS_SUCCESSFUL)
  {
    printf("unexpected task_create status code: %d\n", status);
    exit(1);
  }

  status = rtems_task_start(daq_task_id, DaqTask, 0);
  if (status != RTEMS_SUCCESSFUL)
  {
    printf("unexpected task_start status code: %d\n", status);
    exit(1);
  }
}

rtems_task DaqTask(rtems_task_argument ignored)
{
  rtems_id RM_period_id;
  printf("DaqTask()\n");

  rtems_status_code status;
  status = rtems_rate_monotonic_create(
    rtems_build_name('D', 'A', 'Q', '1'),
    &RM_period_id);
  if (status != RTEMS_SUCCESSFUL)
  {
    printf("unexpected rate_monotic_create status: %d\n", status);
    exit(1);
  }

  printf("starting fake DAQ loop...\n");
  const int max_loop = 10000;

  for (int i = 0; i < max_loop; i++)
  {
    // set the interval in units of ticks (1ms or 100us or whatever in init.c)
    status = rtems_rate_monotonic_period(RM_period_id, 10);
    if (status == RTEMS_TIMEOUT)
    {
      printf("daq timeout\n");
    }
    else if (status != RTEMS_SUCCESSFUL)
    {
      printf("unexpected rate_monotonic_period: %d\n", status);
      exit(1);
    }

    // let's say it's 1.42ms of work to do this task
    rtems_counter_delay_nanoseconds(1420000);
  }
  printf("done with daq task\n");
  rtems_task_exit();
  exit(0);
}
