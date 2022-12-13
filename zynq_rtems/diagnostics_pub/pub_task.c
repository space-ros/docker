#include <rtems.h>
#include <stdlib.h>

rtems_task PubTask(rtems_task_argument ignored)
{
  rtems_id RM_period_id;
  printk("PubTask()\n");

  rtems_status_code status;
  status = rtems_rate_monotonic_create(
    rtems_build_name('P', 'U', 'B', '1'),
    &RM_period_id);
  if (status != RTEMS_SUCCESSFUL)
  {
    printk("unexpected rate_monotic_create status: %d\n", status);
    exit(1);
  }

  while (1)
  {
    status = rtems_rate_monotonic_period(RM_period_id, 1000);
    if (status != RTEMS_SUCCESSFUL)
    {
      printk("unexpected rate_monotonic_period: %d\n", status);
      exit(1);
    }
    rtems_interval t = rtems_clock_get_ticks_since_boot();
    printk("PubTask ready: %d\n", t);

    /*
    status = rtems_clock_get_tod( &time );
    directive_failed( status, "rtems_clock_get_tod" );
    print_time( "TA1 - rtems_clock_get_tod       - ", &time, "\n" );
    */

    /*
    gettimeofday(&tv, NULL);
    diag_msg.header.stamp.sec = tv.tv_sec;
    diag_msg.header.stamp.nanosec = tv.tv_usec * 1000;
    */

  }

  rtems_task_exit();
}
