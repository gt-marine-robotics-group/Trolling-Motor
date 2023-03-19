#include "motors_6.h"
#include "remote_control.h"

#include <LapX9C10X.h>
#include <ServoInput.h>
#include <Servo.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <vector>
/*
  GITMRG Nova V1.1 Custom BLDC Thruster Driver

  v1.1 RoboBoat Testing

  Resources
  https://stackoverflow.com/questions/6504211/is-it-possible-to-include-a-library-from-another-library-using-the-arduino-ide
  https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
*/
#define LIMIT_RESISTANCE 60
#define eToK(e) ((int8_t) (((e) * LIMIT_RESISTANCE / 100)))
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

void error_cb(){
  delay(10);
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// PINS -------------------------------------------------------------

// LIGHT TOWER
const int LT_RED_PIN = A4;
const int LT_GRN_PIN = A3;
const int LT_YEL_PIN = A5;
const int LT_BLU_PIN = A6;
int lt_red_state = 0;
int lt_grn_state = 0;
int lt_yel_state = 0;
int lt_blu_state = 0;

//// ESTOP
//const int ESTOP_OUT_PIN = A8;
//const int ESTOP_SIG_PIN = A9;

// VARS -------------------------------------------------------------
const int throttleMax = 100;
int m_signal = 0;
bool debug = true;
int control_state = 1; // 0 - KILLED | 1 - STANDBY | 2 - MANUAL | 3 - AUTONOMOUS | 4 - AUXILIARY
// add a blinking delay for restoring power after kill state
unsigned long loop_time = 0;
unsigned long last_time = 0;

// DEVICES ----------------------------------------------------------
// MOTORS
Motor6 motors{};

// RC INPUTS
RemoteControl rc {};

//Nick Code :^[
// ROS GLOBAL VARIABLES

int estop_state; // ESTOP HIGH or LOW (LOW = KILLED)

int ros_cmd_a;
int ros_cmd_b;
int ros_cmd_c;
int ros_cmd_d;
int ros_cmd_e;
int ros_cmd_f;

// FUNCTIONS --------------------------------------------------------

namespace std {
void __throw_bad_alloc() {
  Serial.println("Unable to allocate memory");
}
void __throw_length_error( char const*e ) {
  Serial.print("Length Error :"); Serial.println(e);
}
}

// ESTOP MONITOR
//void setup_estop() {
//  pinMode(ESTOP_OUT_PIN, OUTPUT);
//  pinMode(ESTOP_SIG_PIN, INPUT_PULLUP);
//  digitalWrite(ESTOP_OUT_PIN, LOW);
//}
//
//void read_estop() {
//  estop_state = digitalRead(ESTOP_SIG_PIN);
//}


// LIGHT TOWER VARS AND FUNCTIONS
// TODO: Split into separate library

void setup_lt() {
  pinMode(LT_RED_PIN, OUTPUT);
  pinMode(LT_YEL_PIN, OUTPUT);
  pinMode(LT_GRN_PIN, OUTPUT);
  pinMode(LT_BLU_PIN, OUTPUT);
}

void set_lt(bool r, bool y, bool g, bool b) {
  digitalWrite(LT_RED_PIN, r);
  digitalWrite(LT_YEL_PIN, y);
  digitalWrite(LT_GRN_PIN, g);
  digitalWrite(LT_BLU_PIN, b);
}

void set_light(int pin, int flash) {
  bool light_on =  false;
  if (flash == 0) {
    light_on = false;
  }
  else if (flash == 1) {
    light_on = true;
  }
  else if (flash == 2) {
    if (loop_time % 500 <= 200) {
      light_on = true;
    }
  }
  else if (flash == 3) {
    if (loop_time % 1000 <= 400) {
      light_on = true;
    }
  }
  digitalWrite(pin, light_on);
}

void run_lt(int r, int y, int g, int b) {
  set_light(LT_RED_PIN, r);
  set_light(LT_YEL_PIN, y);
  set_light(LT_GRN_PIN, g);
  set_light(LT_BLU_PIN, b);  
}

void cfg_lt(int r, int y, int g, int b){
  lt_red_state = r;
  lt_yel_state = y;
  lt_grn_state = g;
  lt_blu_state = b;
}

//https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_publisher/micro-ros_publisher.ino
//https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_subscriber/micro-ros_subscriber.ino
//rcl_publisher_t vehicle_state_pub;
//rcl_subscription_t subscriber;
// OG: A: left rear, B: left front, C: right front, D: right rear
// New: A: right rear, B: right front, C: left front, D: left rear
rcl_subscription_t motor_a_sub; // A (left_rear)
rcl_subscription_t motor_b_sub; //Nick Code  :C // B (left_front)
rcl_subscription_t motor_c_sub; // C (right_front)
rcl_subscription_t motor_d_sub; // D (right_rear)
rcl_subscription_t motor_e_sub; // E
rcl_subscription_t motor_f_sub; // F

rclc_executor_t executor;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 msg_a;
std_msgs__msg__Int32 msg_b;
std_msgs__msg__Int32 msg_c;
std_msgs__msg__Int32 msg_d;
std_msgs__msg__Int32 msg_e;
std_msgs__msg__Int32 msg_f;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

//void subscription_callback(const void * msgin)
//{  
//  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
//  RCSOFTCHECK(rcl_publish(&vehicle_state_pub, &msg, NULL));
//}
std_msgs__msg__Float32 state_msg;
//void vehicle_state_publish(float vehicle_state)
//{
//  state_msg.data = vehicle_state;
//  RCSOFTCHECK(rcl_publish(&vehicle_state_pub, &msg, NULL));
//}

//Nick Code :^(

float limit_coefficient = -1 * .9;

void left_rear_callback(const void * msgin) 
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_a = int(val * 100 * limit_coefficient);
//  Serial.print("ros_left_rear_thrust: ");
//  Serial.println(ros_left_rear_thrust);
}

void left_middle_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_b = 1 * val * 100 * limit_coefficient;
  ros_cmd_e = -1 * val * 100 * limit_coefficient;
}

void left_front_callback(const void * msgin) 
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_c = val * 100 * limit_coefficient;
//  Serial.print("ros_left_front_thrust: ");
//  Serial.println(ros_left_front_thrust);
}

void right_front_callback(const void * msgin) 
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_d = val * 100 * limit_coefficient;
//  Serial.print("ros_right_front_thrust: ");
//  Serial.println(ros_right_front_thrust);
}

//void right_middle_callback(const void * msgin)
//{
//  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
//  float val = msg->data;
//  ros_cmd_e = val * 100 * limit_coefficient;
//}

void right_rear_callback(const void * msgin) 
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_f = val * 100 * limit_coefficient;
//  Serial.print("ros_right_rear_thrust: ");
//  Serial.println(ros_right_rear_thrust);
}

bool ros_create_entities() {
  // Initialize micro-ROS allocator
  
  delay(1000);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // create node
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = (size_t)(12);
  RCCHECK(rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support, &node_ops));
  // create publisher
//  RCCHECK(rclc_publisher_init_default(
//    &vehicle_state_pub,
//    &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//    "/wamv/nova/mode"));
//  
  // create subscriber
//  RCCHECK(rclc_subscription_init_default(
//    &subscriber,
//    &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//    "micro_ros_arduino_subscriber"));

  //Nick Code >:-(

  // create thrust subscribers
  RCCHECK(rclc_subscription_init_default(
    &motor_a_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/wamv/thrusters/left_rear_thrust_cmd"));
  RCCHECK(rclc_subscription_init_default(
    &motor_b_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/wamv/thrusters/left_middle_thrust_cmd"));
  RCCHECK(rclc_subscription_init_default(
    &motor_c_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/wamv/thrusters/left_front_thrust_cmd"));
  RCCHECK(rclc_subscription_init_default(
    &motor_d_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/wamv/thrusters/right_front_thrust_cmd"));
//  RCCHECK(rclc_subscription_init_default(
//    &motor_e_sub,
//    &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//    "/wamv/thrusters/right_middle_thrust_cmd"));
  RCCHECK(rclc_subscription_init_default(
    &motor_f_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/wamv/thrusters/right_rear_thrust_cmd"));
   
  // create timer,
  // const unsigned int timer_timeout = 1000;

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator)); // Increment this for more subs
  // RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg0, &vehicle_state_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(&executor, &motor_a_sub, &msg_a, &left_rear_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_b_sub, &msg_b, &left_middle_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_c_sub, &msg_c, &left_front_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_d_sub, &msg_d, &right_front_callback, ON_NEW_DATA));
//  RCCHECK(rclc_executor_add_subscription(&executor, &motor_e_sub, &msg_e, &right_middle_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_f_sub, &msg_f, &right_rear_callback, ON_NEW_DATA));
  
  msg.data = 0;
  msg_a.data = 0;
  msg_b.data = 0;
  msg_c.data = 0;
  msg_d.data = 0;
  msg_e.data = 0;
  msg_f.data = 0;
  return true;
}

void ros_destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  //rcl_publisher_fini(&vehicle_state_pub, &node);
  rcl_subscription_fini(&motor_a_sub, &node);
  rcl_subscription_fini(&motor_b_sub, &node);
  rcl_subscription_fini(&motor_c_sub, &node);
  rcl_subscription_fini(&motor_d_sub, &node);
  rcl_subscription_fini(&motor_e_sub, &node);
  rcl_subscription_fini(&motor_f_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void exec_mode(int mode, bool killed) {
  // Publish Vehicle State
  std_msgs__msg__Float32 msg_x;
//  //msg_x.data = mode;
//  msg_x.data = ros_cmd_b;
  
  // Vehicle Logic
  if (killed) {
    // TODO: Listen for killed on actual E-stop circuit in case of manual shutoff
    // TODO: Add a time delay before resuming from killed state with blink
    cfg_lt(1, 0, 0, 0);
  }
  else {
    if (mode + 1 == RemoteControl::ControlState::autonomous){ // AUTONOMOUS
      ros_handler();
      std::vector<int> commands = {ros_cmd_a, ros_cmd_b, ros_cmd_c, ros_cmd_d, ros_cmd_e, ros_cmd_f};
      motors.write_from_ros(commands);
      msg_x.data = ros_cmd_a;
      delay(20);
//      msg_x.data = 2;
    }
    else if (mode + 1 == RemoteControl::ControlState::calibration) { // CALIBRATION
      rc.check_calibration_ready();
      cfg_lt(0, 2, 0, 0);
    }
    else if (mode + 1 == RemoteControl::ControlState::remote_control) { // REMOTE CONTROL
      // set_motor_6x();
      std::vector<int> cmds = motors.get_rc_motor_cmds(rc);
      cfg_lt(0, 1, 0, 0);
      Serial.println("Throttle set");
      // std::vector<int> commands = {rc_cmd_a, rc_cmd_b, rc_cmd_c, rc_cmd_d, rc_cmd_e, rc_cmd_f};
      motors.write_from_rc(cmds);
//      Serial.println(String(rc_cmd_d));
    }
    else {
      Serial.println("Error, mode not supported");
    }
  }
}

void setup() {
  Serial.begin(9600);
  // loop_time = millis();
  
  delay(100);
  Serial.println("NOVA MOTOR STARTING...");
  Serial.println("SETTING UP LIGHT TOWER...");
//  setup_estop();
//  setup_lt();

  Serial.println("INITIALIZING MOTOR CONTROLLERS...");

  Serial.println("CALIBRATING CONTROLLER...");
  rc.calibrate();

  Serial.println("MOTOR SETUP");
  motors.motors_setup(Motor6::default_pins);
  delay(7000);

  Serial.println("============= CALIBRATION COMPLETE ===============");
  delay(500);
  set_microros_transports();
  
  state = WAITING_AGENT;

  Serial.println("==================================================");
  Serial.println("============ NOVA MOTOR INIT COMPLETE ============");
  Serial.println("==================================================");
}

void zero_ros_cmds() {
  ros_cmd_a = 0;
  ros_cmd_b = 0;
  ros_cmd_c = 0;
  ros_cmd_d = 0;
  ros_cmd_e = 0;
  ros_cmd_f = 0;
}


void ros_handler() {
  bool created = false;
  switch (state) {
    case WAITING_AGENT:
      cfg_lt(0, 0, 3, 0);
      zero_ros_cmds();
      motors.zero_all_motors();
      EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      
      break;
    case AGENT_AVAILABLE:
      cfg_lt(0, 0, 2, 0);
      zero_ros_cmds();
      motors.zero_all_motors();
      created = ros_create_entities();
      state = (true == created) ? AGENT_CONNECTED : WAITING_AGENT;
      delay(100);
      if (state == WAITING_AGENT) {
        ros_destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      cfg_lt(0, 0, 1, 0);
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      break;
    case AGENT_DISCONNECTED:
      cfg_lt(3, 0, 3, 0);
      zero_ros_cmds();
      motors.zero_all_motors();
      ros_destroy_entities();
      delay(100);
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}

// TODO: Refactor to use control_state
bool boat_killed = false;

void loop() {
  // Get loop time
  loop_time = millis();
  // E-Stop
//  read_estop();
  // Polling R/C commands
  // read_rc();
  rc.read();
  // Execute based on mode
  boat_killed = false;
  exec_mode(rc.get_ctr_state(), boat_killed);
  // Update light tower
  if (estop_state == true) {
    lt_red_state = 1;
  }
  else {
    lt_red_state = 0;
  }
  run_lt(lt_red_state, lt_yel_state, lt_grn_state, lt_blu_state);
  if (state == AGENT_CONNECTED) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
}
