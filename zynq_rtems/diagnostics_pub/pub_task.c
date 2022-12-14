#include <rtems.h>
#include <stdio.h>
#include <stdlib.h>
#include <zenoh-pico.h>
#include "diagnostic_msgs/msg/diagnostic_array.h"
#include "diagnostic_msgs/msg/diagnostic_status.h"
#include "diagnostic_msgs/msg/key_value.h"
#include "rosidl_runtime_c/string_functions.h"

static rtems_id pub_task_id;
rtems_task PubTask(rtems_task_argument ignored);

void pub_init()
{
  // start the rate-monotonic (periodic) task
  rtems_status_code status;
  status = rtems_task_create(
    rtems_build_name('P', 'U', 'B', '1'),
    123, // task priority
    32 * 1024, // stack size
    RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES,
    &pub_task_id
  );
  if (status != RTEMS_SUCCESSFUL)
  {
    printf("unexpected task_create status code: %d\n", status);
    exit(1);
  }

  status = rtems_task_start(pub_task_id, PubTask, 0);
  if (status != RTEMS_SUCCESSFUL)
  {
    printf("unexpected task_start status code: %d\n", status);
    exit(1);
  }

}

void print_zid(const z_id_t *id, void *ctx)
{
    (void)(ctx);
    printf(" ");
    for (int i = 15; i >= 0; i--) {
        printf("%02X", id->id[i]);
    }
    printf("\n");
}

bool cdr_align(uint8_t **wptr, const uint8_t * const buf_end, const int alignment)
{
  /*
  printf("cdr_align(%d)\n", alignment);
  printf("  wptr = 0x%08x\n", (unsigned)wptr);
  printf("  *wptr = 0x%08x\n", (unsigned)*wptr);
  */
  const int offset = ((uint64_t)(*wptr)) % alignment;
  if (offset == 0)
  {
    //printf("  already aligned!\n");
    // hooray, we're already aligned!
    return true;
  }
  const int padding_length = alignment - offset;
  //printf("  offset = %d, padding_length = %d\n", offset, padding_length);
  if ((*wptr) + padding_length >= buf_end)
  {
    printf("woah! padding of %d tried to overflow a buffer.\n", padding_length);
    // this would overflow...
    return false;
  }
  /*
  printf("adding padding %d to 0x%08x to align to %d\n",
    padding_length,
    (unsigned)*wptr,
    alignment);
  */
  for (int i = 0; i < padding_length; i++)
  {
    //printf("  writing padding byte %d, *wptr = 0x%08x\n", i, (unsigned)*wptr);
    // let's do this in two steps so it's easier to follow:
    **wptr = 0;  // write a zero to the buffer
    (*wptr)++;  // advance the write pointer
  }
  return true;
}

bool cdr_serialize_u8(uint8_t **wptr, const uint8_t * const buf_end, const uint8_t u)
{
  if (!wptr || !(*wptr))
    return false;

  if (*(wptr) >= buf_end)
    return false;

  // first, copy in the string length, including the terminator
  **wptr = u;
  (*wptr)++;
  return true;
}

bool cdr_serialize_u32(uint8_t **wptr, const uint8_t * const buf_end, const uint32_t u)
{
  if (!wptr || !(*wptr))
    return false;

  // check for worst-case alignment
  if (*(wptr) + 8 >= buf_end)
    return false;

  if (!cdr_align(wptr, buf_end, 4))
    return false;

  // first, copy in the string length, including the terminator
  **((uint32_t **)wptr) = u;
  *wptr += 4;
  return true;
}

bool cdr_serialize_string(uint8_t **wptr, const uint8_t * const buf_end, const char *s)
{
  if (!wptr || !(*wptr))
    return false;

  if (!cdr_align(wptr, buf_end, 4))
    return false;

  const int32_t len = s ? (int32_t)strlen(s) : 0;
  // add 4 to overflow estimate, to assume worst-case alignment happened
  if (*(wptr) + len + 4 + 4 >= buf_end)
  {
    printf("not enough space to serialize the string\n");
    return false;
  }

  // first, copy in the string length, including the terminator
  **((uint32_t **)wptr) = len + 1;
  *wptr += 4;

  // if it's an empty string, just write a NULL and bail
  if (!s)
  {
    **wptr = 0;
    (*wptr)++;
    return true;
  }

  // now copy in the string
  memcpy(*wptr, s, len + 1);
  *wptr += len + 1;

  return true;
}

bool cdr_serialize_diagnostic_array(
  uint8_t **wptr,
  const uint8_t *buf_end,
  const diagnostic_msgs__msg__DiagnosticArray * const diag_msg)
{
  // first i32 is the timestamp seconds
  // next u32 is the timestamp nanosec
  // String frame_id
  cdr_serialize_u32(wptr, buf_end, (uint32_t)diag_msg->header.stamp.sec);
  cdr_serialize_u32(wptr, buf_end, diag_msg->header.stamp.nanosec);
  cdr_serialize_string(wptr, buf_end, diag_msg->header.frame_id.data);

  cdr_serialize_u32(wptr, buf_end, diag_msg->status.size);  // sequence length
  for (int status_idx = 0; status_idx < diag_msg->status.size; status_idx++)
  {
    const diagnostic_msgs__msg__DiagnosticStatus * const status =
      &diag_msg->status.data[status_idx];

    // serialize the status summary strings
    cdr_serialize_u8(wptr, buf_end, status->level);
    cdr_serialize_string(wptr, buf_end, status->name.data);
    cdr_serialize_string(wptr, buf_end, status->message.data);
    cdr_serialize_string(wptr, buf_end, status->hardware_id.data);

    // serialize the KeyValue sequence
    cdr_serialize_u32(wptr, buf_end, status->values.size);
    for (int kv_idx = 0; kv_idx < status->values.size; kv_idx++)
    {
      const diagnostic_msgs__msg__KeyValue * const kv = &status->values.data[kv_idx];
      cdr_serialize_string(wptr, buf_end, kv->key.data);
      cdr_serialize_string(wptr, buf_end, kv->value.data);
    }
  }
  return true;
}

rtems_task PubTask(rtems_task_argument ignored)
{
  rtems_id RM_period_id;
  printf("PubTask()\n");

  rtems_status_code status;
  status = rtems_rate_monotonic_create(
    rtems_build_name('P', 'U', 'B', '1'),
    &RM_period_id);
  if (status != RTEMS_SUCCESSFUL)
  {
    printf("unexpected rate_monotic_create status: %d\n", status);
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
    z_string_make("udp/10.0.42.1:7447"));

  printf("Opening zenoh session...\n");
  z_owned_session_t s = z_open(z_move(config));
  if (!z_check(s)) {
    printf("Unable to open session!\n");
    exit(2);
  }
  printf("Zenoh session opened.\n");

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

  z_owned_publisher_t pub = z_declare_publisher(z_loan(s), z_keyexpr("rt/diagnostics"), NULL);
  if (!z_check(pub)) {
    printf("Unable to declare publisher\n");
    exit(1);
  }

  diagnostic_msgs__msg__DiagnosticArray diag_msg;
  diagnostic_msgs__msg__DiagnosticArray__init(&diag_msg);
  // diagnostic status key-values from the Turtlebot4 repo:
  //   - Battery percentage: 42.00
  //   - Battery Voltage: 14.42
  //   - Wheel Status: Enabled
  //   - Docked Status: Undocked
  //   - Hazard Detection: No hazards detected
  diag_msg.header.stamp.sec = 42;
  diag_msg.header.stamp.nanosec = 43;
  rosidl_runtime_c__String__assign(&diag_msg.header.frame_id, "/robot");
  diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(&diag_msg.status, 1);
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].name, "Create3");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].message, "Everything is awesome");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].hardware_id, "12345678");
  diag_msg.status.data[0].level = diagnostic_msgs__msg__DiagnosticStatus__OK;
  diagnostic_msgs__msg__KeyValue__Sequence__init(&diag_msg.status.data[0].values, 5);
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].values.data[0].key, "Battery Percentage");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].values.data[0].value, "42.00");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].values.data[1].key, "Battery Voltage");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].values.data[1].value, "14.42");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].values.data[2].key, "Wheel Status");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].values.data[2].value, "Enabled");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].values.data[3].key, "Docked Status");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].values.data[3].value, "Undocked");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].values.data[4].key, "Hazard Detection");
  rosidl_runtime_c__String__assign(&diag_msg.status.data[0].values.data[4].value, "No hazards detected");

  // wait a bit for our publisher registration to take effect
  // (we need to let some kernel switches happen)
  sleep(1);

  printf("sending a few messages...\n");
  uint8_t msg_buf[512] = {0};
  const uint8_t *buf_end = msg_buf + sizeof(msg_buf);
  const int num_pub = 10;

  for (int i = 0; i < num_pub; i++)
  {
    // set the interval in units of 1ms ticks
    status = rtems_rate_monotonic_period(RM_period_id, 500);
    if (status == RTEMS_TIMEOUT)
    {
      printf("timeout\n");
    }
    else if (status != RTEMS_SUCCESSFUL)
    {
      printf("unexpected rate_monotonic_period: %d\n", status);
      exit(1);
    }

    struct timespec ts;
    rtems_clock_get_monotonic(&ts);
    printf("%d.%09d\n", ts.tv_sec, ts.tv_nsec);
    diag_msg.header.stamp.sec = ts.tv_sec;
    diag_msg.header.stamp.nanosec = ts.tv_nsec * 1000;

    //rtems_interval t = rtems_clock_get_ticks_since_boot();
    //printk("PubTask ready: %d\n", t);

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

    // CDR header (little-endian encoding flag)
    uint8_t *wptr = msg_buf;
    *(wptr++) = 0;
    *(wptr++) = 1;
    *(wptr++) = 0;
    *(wptr++) = 0;

    cdr_serialize_diagnostic_array(&wptr, buf_end, &diag_msg);

    int msg_len = (int)(wptr - msg_buf);
    //printf("msg_len: %d\n", msg_len);

    /*
    for (int j = 0; j < msg_len; j++) {
      printf("  %04d: 0x%02x\n", j, msg_buf[j]);
    }
    */

    z_publisher_put_options_t options = z_publisher_put_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_OCTET_STREAM, NULL);
    z_publisher_put(z_loan(pub), msg_buf, msg_len, &options);
  }

  z_undeclare_publisher(z_move(pub));

  // Stop read and lease tasks for zenoh-pico
  printf("Stopping read and lease tasks...\n");
  zp_stop_read_task(z_session_loan(&s));
  zp_stop_lease_task(z_session_loan(&s));

  printf("Closing zenoh session...\n");
  //z_close(z_move(s)); // not sure why, but this hangs...
  printf("Done. Goodbye.\n");

  //rtems_task_exit();
  exit(0);
}
