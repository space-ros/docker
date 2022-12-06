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
#include <zenoh-pico.h>

static void my_on_exit(int exit_code, void *arg)
{
  rtems_printer printer;
  (void)arg; // suppress warning
  rtems_print_printer_printf(&printer);
  rtems_stack_checker_report_usage_with_plugin(&printer);
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

void print_zid(const z_id_t *id, void *ctx) {
    (void)(ctx);
    printf(" ");
    for (int i = 15; i >= 0; i--) {
        printf("%02X", id->id[i]);
    }
    printf("\n");
}

rtems_task Init(
  rtems_task_argument ignored
)
{
  rtems_bsd_setlogpriority("debug");
  printk("\nInit() start\n\n");
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

  char *ifcfg_print[] = {
    "ifconfig",
    NULL
  };
  exit_code = rtems_bsd_command_ifconfig(RTEMS_BSD_ARGC(ifcfg_print), ifcfg_print);
  if (exit_code != EX_OK)
  {
    printk("ifconfig exit code: %d\n", exit_code);
    exit(1);
  }
  const char *iface_name = "cgem3";

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
  exit_code = rtems_bsd_command_ifconfig(RTEMS_BSD_ARGC(ifcfg_print), ifcfg_print);
  if (exit_code != EX_OK)
  {
    printk("ifconfig exit code: %d\n", exit_code);
    exit(1);
  }
  assert(exit_code == EX_OK);

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

  exit_code = rtems_bsd_command_netstat(RTEMS_BSD_ARGC(route_print), route_print);
  if (exit_code != EX_OK)
  {
    printk("print routes exit code: %d\n", exit_code);
    exit(1);
  }

  z_owned_config_t config = z_config_default();
  zp_config_insert(
    z_config_loan(&config),
    Z_CONFIG_MODE_KEY,
    z_string_make("client"));

  zp_config_insert(
    z_config_loan(&config),
    Z_CONFIG_PEER_KEY,
    z_string_make("tcp/10.0.42.1:7447"));

  printk("Opening zenoh session...\n");
  z_owned_session_t s = z_open(z_move(config));
  if (!z_check(s)) {
    printk("Unable to open session!\n");
    exit(2);
  }
  printk("Zenoh session opened.\n");

  // Start read and lease tasks for zenoh-pico
  if (zp_start_read_task(z_session_loan(&s), NULL) < 0 || zp_start_lease_task(z_session_loan(&s), NULL) < 0) {
      printf("Unable to start read and lease tasks");
      exit(1);
  }
  z_id_t self_id = z_info_zid(z_session_loan(&s));
  printf("Own ID:");
  print_zid(&self_id, NULL);

  printf("Routers IDs:\n");
  z_owned_closure_zid_t callback = z_closure(print_zid);
  z_info_routers_zid(z_loan(s), z_move(callback));

  // `callback` has been `z_move`d just above, so it's safe to reuse the variable,
  // we'll just have to make sure we `z_move` it again to avoid mem-leaks.
  printf("Peers IDs:\n");
  z_owned_closure_zid_t callback2 = z_closure(print_zid);
  z_info_peers_zid(z_loan(s), z_move(callback2));

  // Stop read and lease tasks for zenoh-pico
  printf("Stopping read and lease tasks...\n");
  zp_stop_read_task(z_session_loan(&s));
  zp_stop_lease_task(z_session_loan(&s));

  printf("Closing zenoh session...\n");
  //z_close(z_move(s)); // not sure why, but this hangs...
  printf("Done. Goodbye.\n");
  exit(0);
}
