// #include <LapX9C10X.h>
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
/*
  GITMRG Nova V1.1 Custom BLDC Thruster Driver

  v1.1 RoboBoat Testing

  Resources
  https://stackoverflow.com/questions/6504211/is-it-possible-to-include-a-library-from-another-library-using-the-arduino-ide
  https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
*/
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { return false; } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }
#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)

void error_cb() {
  delay(10);
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// class Motor {
//   private:
//     int dirOnePin; // BWD
//     int dirTwoPin; //FWD
//     int throUdPin;
//     int throIncPin;
//     int throCsPin;

//     LapX9C10X *throttle;

//     void setDirection(int d) {
//       if (d == 0) { //OFF
//         digitalWrite(dirOnePin, LOW);
//         digitalWrite(dirTwoPin, LOW);
//       }
//       else if (d > 0) { //FWD
//         digitalWrite(dirOnePin, LOW);
//         digitalWrite(dirTwoPin, HIGH);
//       }
//       else if (d < 0) { //BWD
//         digitalWrite(dirOnePin, HIGH);
//         digitalWrite(dirTwoPin, LOW);
//       }
//     }
//   public:
//     Motor(uint8_t throIncPin, uint8_t throUdPin, uint8_t throCsPin, float throRes, int dirOnePin, int dirTwoPin) {
//       this->dirOnePin = dirOnePin;
//       this->dirTwoPin = dirTwoPin;
//       this->throUdPin = throUdPin;
//       this->throIncPin = throIncPin;
//       this->throCsPin = throCsPin;
//       // throttle = new LapX9C10X(throIncPin, throUdPin, throCsPin, throRes);
//     }
//     void init() {
//       throttle->begin(); // Min resistance
//       pinMode(dirOnePin, OUTPUT);
//       pinMode(dirTwoPin, OUTPUT);
//     }
//     bool setThrottle(int throttleValue) { // In terms of resistance
//       setDirection(throttleValue);
//       throttle->set(float(abs(throttleValue)));
//       return true;
//     }
//     void resetThrottle() {
//       throttle->reset(0);
//     }
// };

// PINS -------------------------------------------------------------
// RC INPUT
const uint8_t ORX_AUX1_PIN = 2;  // checks if killed - need to figure out reset check
// const int ORX_GEAR_PIN = ; // Kill switch
const uint8_t ORX_RUDD_PIN = 3;  // yaw
const uint8_t ORX_ELEV_PIN = 4;  // WAM-V translate forward / backward
const uint8_t ORX_AILE_PIN = 5;  // WAM-V translate left / right
const uint8_t ORX_THRO_PIN = 6;

// C - Port Fore      D - Starboard Fore
// B - Port Center    E - Starboard Center
// A - Port Aft       F - Starboard Aft
// MOTOR ALFA
const int A_SIG_PIN = 36;
Servo motor_a;
// MOTOR BRAVO
const int B_SIG_PIN = 37;
Servo motor_b;
// MOTOR CHARLIE
const int C_SIG_PIN = 38;
Servo motor_c;
// MOTOR DELTA
const int D_SIG_PIN = 35;
Servo motor_d;
// MOTOR ECHO
const int E_SIG_PIN = 34;
Servo motor_e;
// MOTOR FOXTROT
const int F_SIG_PIN = 33;
Servo motor_f;


// LIGHT TOWER
const int LT_RED_PIN = 24;
const int LT_GRN_PIN = 25;
const int LT_YEL_PIN = 26;
const int LT_BLU_PIN = 27;
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
int control_state = 1;  // 0 - KILLED | 1 - STANDBY | 2 - MANUAL | 3 - AUTONOMOUS | 4 - AUXILIARY
// add a blinking delay for restoring power after kill state
unsigned long loop_time = 0;
unsigned long last_time = 0;

// DEVICES ----------------------------------------------------------
// MOTORS

// RC INPUTS
ServoInputPin<ORX_AUX1_PIN> orxAux1;  // 3 states - Manual / Paused / Autonomous
// ServoInputPin<ORX_GEAR_PIN> orxGear; // 2 states
ServoInputPin<ORX_RUDD_PIN> orxRudd;  // Continuous
ServoInputPin<ORX_ELEV_PIN> orxElev;  // Continuous
ServoInputPin<ORX_AILE_PIN> orxAile;  // Continuous
ServoInputPin<ORX_THRO_PIN> orxThro;  // Continuous

//Nick Code :^[
// ROS GLOBAL VARIABLES

int cmd_srg;  // Commanded Surge
int cmd_swy;  // Commanded Sway
int cmd_yaw;  // Commanded Yaw
int cmd_ctr;  // Commanded Control State
int cmd_kil;  // Commanded Kill State

int estop_state;  // ESTOP HIGH or LOW (LOW = KILLED)

int rc_cmd_a;
int rc_cmd_b;
int rc_cmd_c;
int rc_cmd_d;
int rc_cmd_e;
int rc_cmd_f;

int ros_cmd_a;
int ros_cmd_b;
int ros_cmd_c;
int ros_cmd_d;
int ros_cmd_e;
int ros_cmd_f;

// FUNCTIONS --------------------------------------------------------

// Check current RC status (in order to minimize time polling)
void read_rc() {
  cmd_srg = orxElev.mapDeadzone(-100, 101, 0.05);
  cmd_swy = orxAile.mapDeadzone(-100, 101, 0.05);
  cmd_yaw = orxRudd.mapDeadzone(-100, 101, 0.05);
  cmd_ctr = orxAux1.map(0, 2);
  cmd_kil = 0;  //orxGear.map(1, 0);
  char buffer[100];
  sprintf(buffer, "RC | SRG: %4i  SWY: %4i  YAW: %4i CTR: %1i KIL: %1i",
          cmd_srg, cmd_swy, cmd_yaw, cmd_ctr, cmd_kil);
  //Serial.println(buffer);
}

// Translate RC input to 4x holonomic motor system
// A - Port Aft, D - Starboard Aft, C - Starboard Fore, B - Port Fore
void set_motor_4x_holo() {
  int a = (cmd_srg + cmd_swy - cmd_yaw);
  int b = (cmd_srg - cmd_swy - cmd_yaw);
  int c = (cmd_srg + cmd_swy + cmd_yaw);
  int d = (cmd_srg - cmd_swy + cmd_yaw);
  float max_val = max(100, max(max(abs(a), abs(b)), max(abs(c), abs(d)))) / 100.0;
  rc_cmd_a = a / max_val;
  rc_cmd_b = b / max_val;
  rc_cmd_c = c / max_val;
  rc_cmd_d = d / max_val;
}

void set_motor_4x_tank() {
  int a = cmd_srg;
  int d = cmd_srg;
  int b = cmd_srg;
  int c = cmd_srg;

  float max_val = max(100, max(max(abs(a), abs(b)), max(abs(c), abs(d)))) / 100.0;
  rc_cmd_a = a / max_val;
  rc_cmd_b = b / max_val;
  rc_cmd_c = c / max_val;
  rc_cmd_d = d / max_val;
}

// Translate RC input to 2 motor system
void set_motor_2x() {
  int a = (cmd_srg - cmd_yaw);
  int d = (cmd_srg + cmd_yaw);
  float max_val = max(100, max(abs(a), abs(d))) / 100;
  rc_cmd_a = a / max_val;
  rc_cmd_b = 0;
  rc_cmd_c = 0;
  rc_cmd_d = d / max_val;
}

// C - Port Fore      D - Starboard Fore
// B - Port Center    E - Starboard Center
// A - Port Aft       F - Starboard Aft
void set_motor_6x() {
  int a = cmd_srg - cmd_yaw;
  int b = -cmd_swy;
  int c = cmd_srg - cmd_yaw;
  int d = cmd_srg + cmd_yaw;
  int e = cmd_swy;
  int f = cmd_srg + cmd_yaw;


  float max_val = max(100,
                      max(max(max(abs(a), abs(b)), max(abs(c), abs(d))), max(abs(e), abs(f))))
                  / 100.0;

  rc_cmd_a = a / max_val;
  rc_cmd_b = b / max_val;
  rc_cmd_c = c / max_val;
  rc_cmd_d = d / max_val;
  rc_cmd_e = e / max_val;
  rc_cmd_f = f / max_val;
}

template<uint8_t bs>
bool calibrate_pin(ServoInputPin<bs> &input_pin) {
  const uint16_t pulse = (uint16_t)input_pin.getPulseRaw();
  // Check + store range min/max
  if (pulse < input_pin.getRangeMin()) {
    input_pin.setRangeMin(pulse);
  } else if (pulse > input_pin.getRangeMax()) {
    input_pin.setRangeMax(pulse);
  }
  char buffer[100];
  sprintf(buffer, "Servo PWM (us) | Min: %4u  Val: %4u  Max: %4u | Range: %4u",
          input_pin.getRangeMin(), pulse, input_pin.getRangeMax(), input_pin.getRange());
  //Serial.println(buffer);
  if (input_pin.getRange() < 50 and input_pin.mapDeadzone(-100, 100, .02) != 0) {
    return false;
  }
  return true;
}

bool calibrate_rc() {
  // Get servo signal pulse length, in microseconds (unfiltered)
  bool e_s = calibrate_pin(orxElev);
  bool a_s = calibrate_pin(orxAile);
  bool r_s = calibrate_pin(orxRudd);
  char buffer[100];
  sprintf(buffer, "Calibrated Values | Elev: %4i  Aile: %4i  Rudd: %4i",
          orxElev.mapDeadzone(-100, 100, 0.1), orxAile.mapDeadzone(-100, 100, 0.1), orxRudd.mapDeadzone(-100, 100, 0.1));
  //Serial.println(buffer);
  return (e_s and a_s and r_s);
}

template<uint8_t bs>
void center_pin(ServoInputPin<bs> &input_pin) {
  int center = input_pin.getRangeCenter();
  input_pin.setRange(center, center);
}

void center_rc() {
  center_pin(orxElev);
  center_pin(orxAile);
  center_pin(orxRudd);
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
  bool light_on = false;
  if (flash == 0) {
    light_on = false;
  } else if (flash == 1) {
    light_on = true;
  } else if (flash == 2) {
    if (loop_time % 500 <= 200) {
      light_on = true;
    }
  } else if (flash == 3) {
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

void cfg_lt(int r, int y, int g, int b) {
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
rcl_subscription_t motor_a_sub;  // A (left_rear)
rcl_subscription_t motor_b_sub;  //Nick Code  :C // B (left_front)
rcl_subscription_t motor_c_sub;  // C (right_front)
rcl_subscription_t motor_d_sub;  // D (right_rear)
rcl_subscription_t motor_e_sub;  // E
rcl_subscription_t motor_f_sub;  // F

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

std_msgs__msg__Float32 state_msg;

void left_rear_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_a = int(val * 100);
}

void left_middle_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_b = 1 * val * 100;
  ros_cmd_e = -1 * val * 100;
}

void left_front_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_c = val * 100;
}

void right_front_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_d = val * 100;
}

//void right_middle_callback(const void * msgin)
//{
//  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
//  float val = msg->data;
//  ros_cmd_e = val * 100 * limit_coefficient;
//}

void right_rear_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_f = val * 100;
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

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));  // Increment this for more subs

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
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

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

int maxF(int throttle) {
  if (throttle < -100) {
    throttle = -100;
  } else if (throttle > 100) {
    throttle = 100;
  }
  return throttle;
}

int throttleToESC(int throttle) {
  throttle = maxF(throttle);
  throttle = throttle * 4 + 1500;
  return throttle;
}

void exec_mode(int mode, bool killed) {
  // Vehicle Logic
  if (killed) {
    // TODO: Listen for killed on actual E-stop circuit in case of manual shutoff
    // TODO: Add a time delay before resuming from killed state with blink
    cfg_lt(1, 0, 0, 0);
  } else {
    if (mode == 0) {  // AUTONOMOUS
      ros_handler();
      motor_a.writeMicroseconds(throttleToESC(ros_cmd_a));
      motor_b.writeMicroseconds(throttleToESC(ros_cmd_b));
      motor_c.writeMicroseconds(throttleToESC(ros_cmd_c));
      motor_d.writeMicroseconds(throttleToESC(ros_cmd_d));
      motor_e.writeMicroseconds(throttleToESC(ros_cmd_e));
      motor_f.writeMicroseconds(throttleToESC(ros_cmd_f));
      delay(20);
    } else if (mode == 1) {  // CALIBRATION
      calibrate_rc();
      cfg_lt(0, 2, 0, 0);
    } else if (mode == 2) {  // REMOTE CONTROL
      set_motor_6x();
      cfg_lt(0, 1, 0, 0);
      Serial.println("Throttle set");
      motor_a.writeMicroseconds(throttleToESC(rc_cmd_a));
      motor_b.writeMicroseconds(throttleToESC(rc_cmd_b));
      motor_c.writeMicroseconds(throttleToESC(rc_cmd_c));
      motor_d.writeMicroseconds(throttleToESC(rc_cmd_d));
      motor_e.writeMicroseconds(throttleToESC(rc_cmd_e));
      motor_f.writeMicroseconds(throttleToESC(rc_cmd_f));
    } else {
      Serial.println("Error, mode not supported");
    }
  }
}

void setup() {
  Serial.begin(9600);
  loop_time = millis();

  delay(100);
  Serial.println("NOVA MOTOR STARTING...");
  Serial.println("SETTING UP AND TESTING LIGHT TOWER...");
  //  setup_estop();
  setup_lt();
  set_lt(1, 1, 1, 1);
  delay(500);

  Serial.println("CALIBRATING CONTROLLER...");
  // digitalWrite(LT_RED_PIN, HIGH);
  bool mode_ready = false;
  bool calibration_ready = false;
  int calibration_zero_check = 0;
  center_rc();
  while (not mode_ready or not calibration_ready or calibration_zero_check < 15) {
    loop_time = millis();
    run_lt(2, 2, 0, 0);
    //set_lt(1, 1, 1, 1);
    read_rc();
    if (cmd_ctr == 1) {
      mode_ready = true;
    }
    calibration_ready = calibrate_rc();
    if (abs(cmd_srg) + abs(cmd_swy) + abs(cmd_yaw) <= 4) {
      calibration_zero_check += 1;
    } else {
      calibration_zero_check = 0;
    }
  }
  Serial.println("============= CALIBRATION COMPLETE ===============");

  Serial.println("INITIALIZING MOTORS...");
  motor_a.attach(A_SIG_PIN);
  motor_b.attach(B_SIG_PIN);
  motor_c.attach(C_SIG_PIN);
  motor_d.attach(D_SIG_PIN);
  motor_e.attach(E_SIG_PIN);
  motor_f.attach(F_SIG_PIN);
  motor_a.writeMicroseconds(1500);
  motor_b.writeMicroseconds(1500);
  motor_c.writeMicroseconds(1500);
  motor_d.writeMicroseconds(1500);
  motor_e.writeMicroseconds(1500);
  motor_f.writeMicroseconds(1500);
  delay(2500);

  Serial.println("SETTING UP MICROROS TRANSPORTS...");
  set_microros_transports();

  state = WAITING_AGENT;

  Serial.println("==================================================");
  Serial.println("============ NOVA MOTOR INIT COMPLETE ============");
  Serial.println("==================================================");
}

void zero_all_motors() {
  motor_a.writeMicroseconds(1500);
  motor_b.writeMicroseconds(1500);
  motor_c.writeMicroseconds(1500);
  motor_d.writeMicroseconds(1500);
  motor_e.writeMicroseconds(1500);
  motor_f.writeMicroseconds(1500);
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
      cfg_lt(3, 0, 3, 0);
      zero_ros_cmds();
      zero_all_motors();
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 2)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      cfg_lt(0, 0, 2, 0);
      zero_ros_cmds();
      zero_all_motors();
      created = ros_create_entities();
      state = (true == created) ? AGENT_CONNECTED : WAITING_AGENT;
      delay(100);
      if (state == WAITING_AGENT) {
        ros_destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      cfg_lt(0, 0, 1, 0);
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 4)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      break;
    case AGENT_DISCONNECTED:
      set_lt(1, 0, 1, 0);    
      zero_ros_cmds();
      zero_all_motors();
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
  read_rc();
  // Execute based on mode
  boat_killed = false;
  exec_mode(cmd_ctr, boat_killed);
  // Update light tower
  if (estop_state == true) {
    lt_red_state = 1;
  }
  run_lt(lt_red_state, lt_yel_state, lt_grn_state, lt_blu_state);
  if (state == AGENT_CONNECTED) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
}
