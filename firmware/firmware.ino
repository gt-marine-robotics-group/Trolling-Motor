#include <LapX9C10X.h>
#include <ServoInput.h>

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
  GITMRG Supernova V1.1 Custom Trolling Motor Driver

  Interfaces with PWM Module through digital potentiometer
  PWM Module is interfaced with NV-series 8-speed trolling motor.

  v0.0 - 25 JUN 2022
  by Sean Fish, Alvaro Pelaez
  v1.0 - Latest
  by Sean Fish, Nicholas Graham

  Roadmap
   - Version 0: Arduino Nano - Serial Commuication
   - Version 1: Arduino Due - micro-ROS
   - Version 2: Teensy 4.0 - micro-ROS

  Resources
  https://stackoverflow.com/questions/6504211/is-it-possible-to-include-a-library-from-another-library-using-the-arduino-ide
  https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
*/
#define LIMIT_RESISTANCE 60
#define eToK(e) ((uint8_t) (((e) * LIMIT_RESISTANCE / 100)))
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

class Motor {
  private:
    int dirOnePin; // BWD
    int dirTwoPin; //FWD
    int throUdPin;
    int throIncPin;
    int throCsPin;

    LapX9C10X *throttle;

    void setDirection(int d) {
      if (d == 0) { //OFF
        digitalWrite(dirOnePin, LOW);
        digitalWrite(dirTwoPin, LOW);
      }
      else if (d > 0) { //FWD
        digitalWrite(dirOnePin, LOW);
        digitalWrite(dirTwoPin, HIGH);
      }
      else if (d < 0) { //BWD
        digitalWrite(dirOnePin, HIGH);
        digitalWrite(dirTwoPin, LOW);
      }
    }
  public:
    Motor(uint8_t throIncPin, uint8_t throUdPin, uint8_t throCsPin, float throRes, int dirOnePin, int dirTwoPin) {
      this->dirOnePin = dirOnePin;
      this->dirTwoPin = dirTwoPin;
      this->throUdPin = throUdPin;
      this->throIncPin = throIncPin;
      this->throCsPin = throCsPin;
      throttle = new LapX9C10X(throIncPin, throUdPin, throCsPin, throRes);
    }
    void init() {
      throttle->begin(); // Min resistance
      pinMode(dirOnePin, OUTPUT);
      pinMode(dirTwoPin, OUTPUT);
    }
    bool setThrottle(int throttleValue) { // In terms of resistance
      setDirection(throttleValue);
      throttle->set(float(abs(throttleValue)));
      return true;
    }
    void resetThrottle() {
      throttle->reset(0);
    }
};

// PINS -------------------------------------------------------------
// RC INPUT
const int ORX_AUX1_PIN = 43; // checks if killed - need to figure out reset check
const int ORX_GEAR_PIN = 45; // Kill switch
const int ORX_RUDD_PIN = 47; // yaw
const int ORX_ELEV_PIN = 49; // WAM-V translate forward / backward
const int ORX_AILE_PIN = 51; // WAM-V translate left / right
const int ORX_THRO_PIN = 53;

// MOTOR ALFA
const int A_THRO_CS_PIN = 12;
const int A_THRO_UD_PIN = 11;
const int A_THRO_INC_PIN = 10;
const int A_DIR_SEL0_PIN = 9;
const int A_DIR_SEL1_PIN = 8;
const int A_THRO_RESISTANCE = 66;
// MOTOR BRAVO
const int B_THRO_CS_PIN = 7;
const int B_THRO_UD_PIN = 6;
const int B_THRO_INC_PIN = 5;
const int B_DIR_SEL0_PIN = 4;
const int B_DIR_SEL1_PIN = 3;
const int B_THRO_RESISTANCE = 87;
// MOTOR CHARLIE
const int C_THRO_CS_PIN = 14;
const int C_THRO_UD_PIN = 15;
const int C_THRO_INC_PIN = 16;
const int C_DIR_SEL0_PIN = 17;
const int C_DIR_SEL1_PIN = 18;
const int C_THRO_RESISTANCE = 74;
// MOTOR DELTA
const int D_THRO_CS_PIN = 23;
const int D_THRO_UD_PIN = 25;
const int D_THRO_INC_PIN = 27;
const int D_DIR_SEL0_PIN = 29;
const int D_DIR_SEL1_PIN = 31;
const int D_THRO_RESISTANCE = 100;

// LIGHT TOWER
const int LT_RED_PIN = A4;
const int LT_GRN_PIN = A3;
const int LT_YEL_PIN = A5;
const int LT_BLU_PIN = A6;

// VARS -------------------------------------------------------------
const int THRO_RESISTANCE = LAPX9C10X_X9C104;
const int throttleMax = 50;
int m_signal = 0;
bool debug = true;
int control_state = 1; // 0 - KILLED | 1 - STANDBY | 2 - MANUAL | 3 - AUTONOMOUS | 4 - AUXILIARY
// add a blinking delay for restoring power after kill state
unsigned long loop_time = 0;
unsigned long last_time = 0;

// DEVICES ----------------------------------------------------------
// MOTORS
Motor motor_a(A_THRO_INC_PIN, A_THRO_UD_PIN, A_THRO_CS_PIN, A_THRO_RESISTANCE, A_DIR_SEL0_PIN, A_DIR_SEL1_PIN);
Motor motor_b(B_THRO_INC_PIN, B_THRO_UD_PIN, B_THRO_CS_PIN, B_THRO_RESISTANCE, B_DIR_SEL0_PIN, B_DIR_SEL1_PIN);
Motor motor_c(C_THRO_INC_PIN, C_THRO_UD_PIN, C_THRO_CS_PIN, C_THRO_RESISTANCE, C_DIR_SEL0_PIN, C_DIR_SEL1_PIN);
Motor motor_d(D_THRO_INC_PIN, D_THRO_UD_PIN, D_THRO_CS_PIN, D_THRO_RESISTANCE, D_DIR_SEL0_PIN, D_DIR_SEL1_PIN);

// RC INPUTS
ServoInputPin<ORX_AUX1_PIN> orxAux1; // 3 states - Manual / Paused / Autonomous
ServoInputPin<ORX_GEAR_PIN> orxGear; // 2 states
ServoInputPin<ORX_RUDD_PIN> orxRudd; // Continuous
ServoInputPin<ORX_ELEV_PIN> orxElev; // Continuous
ServoInputPin<ORX_AILE_PIN> orxAile; // Continuous
ServoInputPin<ORX_THRO_PIN> orxThro; // Continuous

//Nick Code :^[
// ROS GLOBAL VARIABLES

int cmd_srg; // Commanded Surge
int cmd_swy; // Commanded Sway
int cmd_yaw; // Commanded Yaw
int cmd_ctr; // Commanded Control State
int cmd_kil; // Commanded Kill State

int rc_cmd_a;
int rc_cmd_b;
int rc_cmd_c;
int rc_cmd_d;

int ros_cmd_a;
int ros_cmd_b;
int ros_cmd_c;
int ros_cmd_d;

// FUNCTIONS --------------------------------------------------------

// Check current RC status (in order to minimize time polling)
void read_rc() {
  cmd_srg = orxElev.mapDeadzone(-100, 101, 0.05);
  cmd_swy = orxAile.mapDeadzone(-100, 101, 0.05);
  cmd_yaw = orxRudd.mapDeadzone(-100, 101, 0.05);
  cmd_ctr = orxAux1.map(0, 3);
  cmd_kil = orxGear.map(1, 0);
  char buffer[100];
  sprintf(buffer, "RC | SRG: %4i  SWY: %4i  YAW: %4i CTR: %1i KIL: %1i", 
    cmd_srg, cmd_swy, cmd_yaw, cmd_ctr, cmd_kil);
  //Serial.println(buffer);
}

// Translate RC input to 4x holonomic motor system
// A - Port Aft, D - Starboard Aft, C - Starboard Fore, B - Port Fore
void set_motor_4x() {
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

template<uint8_t bs>
bool calibrate_pin(ServoInputPin<bs> &input_pin) {
  const uint16_t pulse = (uint16_t) input_pin.getPulseRaw();
  // Check + store range min/max
  if (pulse < input_pin.getRangeMin()) {
    input_pin.setRangeMin(pulse);
  }
  else if (pulse > input_pin.getRangeMax()) {
    input_pin.setRangeMax(pulse);
  }
  char buffer[100];
  sprintf(buffer, "Servo PWM (us) | Min: %4u  Val: %4u  Max: %4u | Range: %4u", 
    input_pin.getRangeMin(), pulse, input_pin.getRangeMax(), input_pin.getRange());
  //Serial.println(buffer);
  if (input_pin.getRange() < 50 and input_pin.mapDeadzone(-100,100, .02) != 0){
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
    orxElev.mapDeadzone(-100, 100, 0.05), orxAile.mapDeadzone(-100, 100, 0.05), orxRudd.mapDeadzone(-100, 100, 0.05));
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

//https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_publisher/micro-ros_publisher.ino
//https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_subscriber/micro-ros_subscriber.ino
rcl_publisher_t vehicle_state_pub;
//rcl_subscription_t subscriber;

rcl_subscription_t motor_a_sub; // A (left_rear)
rcl_subscription_t motor_b_sub; //Nick Code  :C // B (left_front)
rcl_subscription_t motor_c_sub; // C (right_front)
rcl_subscription_t motor_d_sub; // D (right_rear)

rclc_executor_t executor;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 msg_a;
std_msgs__msg__Int32 msg_b;
std_msgs__msg__Int32 msg_c;
std_msgs__msg__Int32 msg_d;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  RCSOFTCHECK(rcl_publish(&vehicle_state_pub, &msg, NULL));
}

//Nick Code :^(

void left_rear_callback(const void * msgin) 
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_a = val * 100;
//  Serial.print("ros_left_rear_thrust: ");
//  Serial.println(ros_left_rear_thrust);
}

void left_front_callback(const void * msgin) 
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_b = val * 100;
//  Serial.print("ros_left_front_thrust: ");
//  Serial.println(ros_left_front_thrust);
}

void right_front_callback(const void * msgin) 
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_c = val * 100;
//  Serial.print("ros_right_front_thrust: ");
//  Serial.println(ros_right_front_thrust);
}

void right_rear_callback(const void * msgin) 
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_d = val * 100;
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
  RCCHECK(rclc_publisher_init_default(
    &vehicle_state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/nova/vehicle_state"));
  
  // create subscriber
//  RCCHECK(rclc_subscription_init_default(
//    &subscriber,
//    &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//    "micro_ros_arduino_subscriber"));

  //Nick Code >:-(

  // create thrust subscribers
  RCCHECK(rclc_subscription_init_default(
    &motor_b_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/wamv/thrusters/left_front_thrust_cmd"));

  RCCHECK(rclc_subscription_init_default(
    &motor_c_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/wamv/thrusters/right_front_thrust_cmd"));

  RCCHECK(rclc_subscription_init_default(
    &motor_a_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/wamv/thrusters/left_rear_thrust_cmd"));

  RCCHECK(rclc_subscription_init_default(
    &motor_d_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/wamv/thrusters/right_rear_thrust_cmd"));
   
  // create timer,
  // const unsigned int timer_timeout = 1000;

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator)); // Increment this for more subs
  // RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg0, &subscription_callback, ON_NEW_DATA));

  //Nick Code :-l
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_a_sub, &msg_a, &left_rear_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_b_sub, &msg_b, &left_front_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_c_sub, &msg_c, &right_front_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_d_sub, &msg_d, &right_rear_callback, ON_NEW_DATA));
  
  msg.data = 0;
  msg_a.data = 0;
  msg_b.data = 0;
  msg_c.data = 0;
  msg_d.data = 0;
  return true;
}

void ros_destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&vehicle_state_pub, &node);
  rcl_subscription_fini(&motor_a_sub, &node);
  rcl_subscription_fini(&motor_b_sub, &node);
  rcl_subscription_fini(&motor_c_sub, &node);
  rcl_subscription_fini(&motor_d_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void exec_mode(int mode, bool killed) {
  // Publish Vehicle State
  std_msgs__msg__Int32 msg_x;
  //msg_x.data = mode;
  msg_x.data = ros_cmd_b;
  RCSOFTCHECK(rcl_publish(&vehicle_state_pub, &msg_x, NULL));
  // Vehicle Logic
  if (killed) {
    // TODO: Listen for killed on actual E-stop circuit in case of manual shutoff
    // TODO: Add a time delay before resuming from killed state with blink
    run_lt(1, 0, 0, 0);
    motor_a.setThrottle(0);
    motor_b.setThrottle(0);
    motor_c.setThrottle(0);
    motor_d.setThrottle(0);
  }
  else {
    motor_a.resetThrottle();
    motor_b.resetThrottle();
    motor_c.resetThrottle();
    motor_d.resetThrottle();
    if (mode == 0){ // AUTONOMOUS
      ros_handler();
      run_lt(0, 0, 1, 0);
      motor_a.setThrottle(eToK(ros_cmd_a));
      motor_b.setThrottle(eToK(ros_cmd_b));
      motor_c.setThrottle(eToK(ros_cmd_c));
      motor_d.setThrottle(eToK(ros_cmd_d));
    }
    else if (mode == 1) { // CALIBRATION
      calibrate_rc();
      run_lt(0, 2, 0, 0);
      motor_a.setThrottle(0);
      motor_b.setThrottle(0);
      motor_c.setThrottle(0);
      motor_d.setThrottle(0);
    }
    else if (mode == 2) { // REMOTE CONTROL
      set_motor_4x();
      run_lt(0, 1, 0, 0);
//      char buffer[100];
//       sprintf(buffer, "Manual | A: %4i  B: %4i  C: %4i D: %4i", 
//      rc_cmd_a, rc_cmd_b, rc_cmd_c, rc_cmd_d);
//      Serial.println(buffer);
      motor_a.setThrottle(eToK(rc_cmd_a));
      motor_b.setThrottle(eToK(rc_cmd_b));
      motor_c.setThrottle(eToK(rc_cmd_c));
      motor_d.setThrottle(eToK(rc_cmd_d));
    }
    else {
      Serial.println("Error, mode not supported");
    }
  }
}

void  setup() {
  set_microros_transports();
  Serial.begin(9600);
  loop_time = millis();
 
  delay(100);
  Serial.println("NOVA MOTOR STARTING...");
  Serial.println("SETTING UP LIGHT TOWER...");
  setup_lt();

  Serial.println("CALIBRATING CONTROLLER...");
  bool mode_ready = false;
  bool calibration_ready = false;
  int calibration_zero_check = 0;
  center_rc();
  while(not mode_ready or not calibration_ready or calibration_zero_check < 15) {
    loop_time = millis();
    run_lt(0, 2, 0, 0);
    read_rc();
    if (cmd_ctr == 1) {
      mode_ready = true;
    }
    calibration_ready = calibrate_rc();
    if (cmd_srg + cmd_swy + cmd_yaw == 0) {
      calibration_zero_check += 1;
    }
    else {
      calibration_zero_check = 0;
    }
  }

  delay(500);
  
  Serial.println("INITIALIZING MOTOR CONTROLLERS...");
  motor_a.init();
  motor_b.init();
  motor_c.init();
  motor_d.init();
  motor_a.setThrottle(0);
  motor_b.setThrottle(0);
  motor_c.setThrottle(0);
  motor_d.setThrottle(0);

  Serial.println("==================================================");
  Serial.println("============ NOVA MOTOR INIT COMPLETE ============");
  Serial.println("==================================================");
}

void zero_all_motors() {
  motor_a.setThrottle(0);
  motor_b.setThrottle(0);
  motor_c.setThrottle(0);
  motor_d.setThrottle(0);
}
void zero_ros_cmds() {
  ros_cmd_a = 0;
  ros_cmd_b = 0;
  ros_cmd_c = 0;
  ros_cmd_d = 0;
}


void ros_handler() {
  switch (state) {
    case WAITING_AGENT:
      run_lt(0, 0, 0, 1);
      zero_ros_cmds();
      EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      
      break;
    case AGENT_AVAILABLE:
      state = (true == ros_create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        ros_destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      zero_ros_cmds();
      ros_destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}

void loop() {
  // Get loop time
  loop_time = millis();
  // Polling R/C commands
  read_rc();
  // Execute based on mode
  // Serial.println("execute");
  exec_mode(cmd_ctr, cmd_kil);
  // exec_mode(0, cmd_kil);
}
