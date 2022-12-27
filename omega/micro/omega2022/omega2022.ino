///////////////////////////odrive
#include <HardwareSerial.h>
#include <ODriveArduino.h>
HardwareSerial& odrive_serial1 = Serial1; ODriveArduino odrive1(odrive_serial1);
#define vMax  2
///////////////////////////odrive

///////////////////////////microROS
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
///////////////////////////microROS

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)\

    rclc_support_t support;
  rcl_node_t node;
  rcl_timer_t timer;
  rclc_executor_t executor;
  rcl_allocator_t allocator;
  rcl_publisher_t publisher;
  std_msgs__msg__Int32 msg;
  bool micro_ros_init_successful;

  enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
  } state;

  rcl_subscription_t left_wheel_sub;
  std_msgs__msg__Float32 left_wheel_msg;
  float L_wheel_speed = 0.00;

  rcl_subscription_t right_wheel_sub;
  std_msgs__msg__Float32 right_wheel_msg;
  float R_wheel_speed = 0.00;

  /////////////////////////////////////
  uint8_t L1_index, R1_index, L2_index, R2_index, T_index;
  float L1_value = 0.00, R1_value = 0.00, L2_value = 0.00, R2_value = 0.00;
  float cal1, cal2, cal3, cal4, calc1, calc2, calc3, calc4, current;
  float cmps = 63.83716;
  ///////////////////////////////////////

  void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
  {
    (void) last_call_time;
    if (timer != NULL) {
    }
  }

  bool create_entities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "MicroController", "", &support));
    RCCHECK(rclc_subscription_init_default(
              &left_wheel_sub,
              &node,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
              "wheel_command_left"));

    RCCHECK(rclc_subscription_init_default(
              &right_wheel_sub,
              &node,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
              "wheel_command_right"));

    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(
              &timer,
              &support,
              RCL_MS_TO_NS(timer_timeout),
              timer_callback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &left_wheel_sub, &left_wheel_msg, &leftwheel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &right_wheel_sub, &right_wheel_msg, &rightwheel_callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    return true;
  }

  void destroy_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_subscription_fini(&left_wheel_sub, &node);
    rcl_subscription_fini(&right_wheel_sub, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
  }

  void leftwheel_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    L_wheel_speed = msg->data;
    float L_cal = L_wheel_speed / (cmps / 100);
    if (L_cal == 0) {
      odrive1.SetVelocity(0, 0);
      odrive1.run_state(0, 8, false);
    }
    else odrive1.SetVelocity(0, -L_cal );
  }

  void rightwheel_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    R_wheel_speed = msg->data;
    float R_cal = R_wheel_speed / (cmps / 100);
    if (R_cal == 0) {
      odrive1.SetVelocity(1, 0);
      odrive1.run_state(1, 8, false);
    }
    else odrive1.SetVelocity(1, R_cal);
  }

  void setup() {
    odrive_serial1.begin(115200, SERIAL_8N1, 17, 16);
    odrive1.SetVelocity(0, 0.00); odrive1.SetVelocity(1, 0.00);
    odrive1.run_state(0, 8, false); odrive1.run_state(1, 8, false);
    set_microros_transports();
    state = WAITING_AGENT;
  }

  void loop() {
    switch (state) {
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
          destroy_entities();
        };
        break;
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
      case AGENT_DISCONNECTED:
        destroy_entities();
        state = WAITING_AGENT;
        break;
      default:
        break;
    }
  }
