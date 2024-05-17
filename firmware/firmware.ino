// #include <LapX9C10X.h>
#include <ServoInput.h>
#include <MS5837.h>
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
#include <std_msgs/msg/float32_multi_array.h>
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
// const uint8_t ORX_AUX1_PIN = 2;  // checks if killed - need to figure out reset check
// // const int ORX_GEAR_PIN = ; // Kill switch
// const uint8_t ORX_RUDD_PIN = 3;  // yaw
// const uint8_t ORX_ELEV_PIN = 4;  // WAM-V translate forward / backward
// const uint8_t ORX_AILE_PIN = 5;  // WAM-V translate left / right
// const uint8_t ORX_THRO_PIN = 6;

// C - Port Fore      D - Starboard Fore
// B - Port Center    E - Starboard Center
// A - Port Aft       F - Starboard Aft
// MOTOR ALFA
const int A_SIG_PIN = 33; // BACK LEFT VERT
Servo motor_a;
// MOTOR BRAVO
const int B_SIG_PIN = 34; // BACK RIGHT VERT
Servo motor_b;
// MOTOR CHARLIE
const int C_SIG_PIN = 35; // FRONT LEFT VERT
Servo motor_c;
// MOTOR DELTA
const int D_SIG_PIN = 36; // FRONT RIGHT VERT
Servo motor_d;
// MOTOR ECHO
const int E_SIG_PIN = 37; // BACK LEFT HORIZ
Servo motor_e;
// MOTOR FOXTROT
const int F_SIG_PIN = 39; // BACK RIGHT HORIZ
Servo motor_f;

const int G_SIG_PIN = 40; // FRONT LEFT HORIZ
Servo motor_g;

const int H_SIG_PIN = 41; // FRONT RIGHT HORIZ
Servo motor_h;

MS5837 depth_sensor;


// LIGHT TOWER
// const int LT_RED_PIN = 24;
// const int LT_GRN_PIN = 25;
// const int LT_YEL_PIN = 26;
// const int LT_BLU_PIN = 27;
// int lt_red_state = 0;
// int lt_grn_state = 0;
// int lt_yel_state = 0;
// int lt_blu_state = 0;

//// ESTOP
//const int ESTOP_OUT_PIN = A8;
//const int ESTOP_SIG_PIN = A9;

// VARS -------------------------------------------------------------
const int throttleMax = 100;
int m_signal = 0;
bool debug = true;
bool depth_sensor_init = false;
int control_state = 1;  // 0 - KILLED | 1 - STANDBY | 2 - MANUAL | 3 - AUTONOMOUS | 4 - AUXILIARY
// add a blinking delay for restoring power after kill state
unsigned long loop_time = 0;
unsigned long last_time = 0;

// DEVICES ----------------------------------------------------------
// MOTORS

// RC INPUTS
// ServoInputPin<ORX_AUX1_PIN> orxAux1;  // 3 states - Manual / Paused / Autonomous
// // ServoInputPin<ORX_GEAR_PIN> orxGear; // 2 states
// ServoInputPin<ORX_RUDD_PIN> orxRudd;  // Continuous
// ServoInputPin<ORX_ELEV_PIN> orxElev;  // Continuous
// ServoInputPin<ORX_AILE_PIN> orxAile;  // Continuous
// ServoInputPin<ORX_THRO_PIN> orxThro;  // Continuous

//Nick Code :^[
// ROS GLOBAL VARIABLES

// int cmd_srg;  // Commanded Surge
// int cmd_swy;  // Commanded Sway
// int cmd_yaw;  // Commanded Yaw
// int cmd_ctr;  // Commanded Control State
// int cmd_kil;  // Commanded Kill State

// int estop_state;  // ESTOP HIGH or LOW (LOW = KILLED)

// int rc_cmd_a;
// int rc_cmd_b;
// int rc_cmd_c;
// int rc_cmd_d;
// int rc_cmd_e;
// int rc_cmd_f;

float ros_cmd_a;
float ros_cmd_b;
float ros_cmd_c;
float ros_cmd_d;
float ros_cmd_e;
float ros_cmd_f;
float ros_cmd_g;
float ros_cmd_h;

int led = 13;

// FUNCTIONS --------------------------------------------------------

// Check current RC status (in order to minimize time polling)
// void read_rc() {
//   cmd_srg = orxElev.mapDeadzone(-100, 101, 0.05);
//   cmd_swy = orxAile.mapDeadzone(-100, 101, 0.05);
//   cmd_yaw = orxRudd.mapDeadzone(-100, 101, 0.05);
//   cmd_ctr = orxAux1.map(0, 2);
//   cmd_kil = 0;  //orxGear.map(1, 0);
//   char buffer[100];
//   sprintf(buffer, "RC | SRG: %4i  SWY: %4i  YAW: %4i CTR: %1i KIL: %1i",
//           cmd_srg, cmd_swy, cmd_yaw, cmd_ctr, cmd_kil);
//   //Serial.println(buffer);
// }

// Translate RC input to 4x holonomic motor system
// A - Port Aft, D - Starboard Aft, C - Starboard Fore, B - Port Fore
// void set_motor_4x_holo() {
//   int a = (cmd_srg + cmd_swy - cmd_yaw);
//   int b = (cmd_srg - cmd_swy - cmd_yaw);
//   int c = (cmd_srg + cmd_swy + cmd_yaw);
//   int d = (cmd_srg - cmd_swy + cmd_yaw);
//   float max_val = max(100, max(max(abs(a), abs(b)), max(abs(c), abs(d)))) / 100.0;
//   rc_cmd_a = a / max_val;
//   rc_cmd_b = b / max_val;
//   rc_cmd_c = c / max_val;
//   rc_cmd_d = d / max_val;
// }

// void set_motor_4x_tank() {
//   int a = cmd_srg;
//   int d = cmd_srg;
//   int b = cmd_srg;
//   int c = cmd_srg;

//   float max_val = max(100, max(max(abs(a), abs(b)), max(abs(c), abs(d)))) / 100.0;
//   rc_cmd_a = a / max_val;
//   rc_cmd_b = b / max_val;
//   rc_cmd_c = c / max_val;
//   rc_cmd_d = d / max_val;
// }

// Translate RC input to 2 motor system
// void set_motor_2x() {
//   int a = (cmd_srg - cmd_yaw);
//   int d = (cmd_srg + cmd_yaw);
//   float max_val = max(100, max(abs(a), abs(d))) / 100;
//   rc_cmd_a = a / max_val;
//   rc_cmd_b = 0;
//   rc_cmd_c = 0;
//   rc_cmd_d = d / max_val;
// }

// C - Port Fore      D - Starboard Fore
// B - Port Center    E - Starboard Center
// A - Port Aft       F - Starboard Aft
// void set_motor_6x() {
//   int a = cmd_srg - cmd_yaw;
//   int b = -cmd_swy;
//   int c = cmd_srg - cmd_yaw;
//   int d = cmd_srg + cmd_yaw;
//   int e = cmd_swy;
//   int f = cmd_srg + cmd_yaw;


//   float max_val = max(100,
//                       max(max(max(abs(a), abs(b)), max(abs(c), abs(d))), max(abs(e), abs(f))))
//                   / 100.0;

//   rc_cmd_a = a / max_val;
//   rc_cmd_b = b / max_val;
//   rc_cmd_c = c / max_val;
//   rc_cmd_d = d / max_val;
//   rc_cmd_e = e / max_val;
//   rc_cmd_f = f / max_val;
// }

// template<uint8_t bs>
// bool calibrate_pin(ServoInputPin<bs> &input_pin) {
//   const uint16_t pulse = (uint16_t)input_pin.getPulseRaw();
//   // Check + store range min/max
//   if (pulse < input_pin.getRangeMin()) {
//     input_pin.setRangeMin(pulse);
//   } else if (pulse > input_pin.getRangeMax()) {
//     input_pin.setRangeMax(pulse);
//   }
//   char buffer[100];
//   sprintf(buffer, "Servo PWM (us) | Min: %4u  Val: %4u  Max: %4u | Range: %4u",
//           input_pin.getRangeMin(), pulse, input_pin.getRangeMax(), input_pin.getRange());
//   //Serial.println(buffer);
//   if (input_pin.getRange() < 50 and input_pin.mapDeadzone(-100, 100, .02) != 0) {
//     return false;
//   }
//   return true;
// }

// bool calibrate_rc() {
//   // Get servo signal pulse length, in microseconds (unfiltered)
//   bool e_s = calibrate_pin(orxElev);
//   bool a_s = calibrate_pin(orxAile);
//   bool r_s = calibrate_pin(orxRudd);
//   char buffer[100];
//   sprintf(buffer, "Calibrated Values | Elev: %4i  Aile: %4i  Rudd: %4i",
//           orxElev.mapDeadzone(-100, 100, 0.1), orxAile.mapDeadzone(-100, 100, 0.1), orxRudd.mapDeadzone(-100, 100, 0.1));
//   //Serial.println(buffer);
//   return (e_s and a_s and r_s);
// }

// template<uint8_t bs>
// void center_pin(ServoInputPin<bs> &input_pin) {
//   int center = input_pin.getRangeCenter();
//   input_pin.setRange(center, center);
// }

// void center_rc() {
//   center_pin(orxElev);
//   center_pin(orxAile);
//   center_pin(orxRudd);
// }

//https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_publisher/micro-ros_publisher.ino
//https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_subscriber/micro-ros_subscriber.ino
//rcl_publisher_t vehicle_state_pub;
//rcl_subscription_t subscriber;

rcl_subscription_t motors_sub;
rcl_publisher_t depth_pub;

rclc_executor_t executor;
std_msgs__msg__Float32MultiArray msg_in;
std_msgs__msg__Float32 msg_depth;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

void motors_callback(std_msgs__msg__Float32MultiArray * msg) {
  // const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  digitalWrite(led, HIGH);
  ros_cmd_a = msg->data.data[0] * 100;
  ros_cmd_b = msg->data.data[1] * 100;
  ros_cmd_c = msg->data.data[2] * 100;
  ros_cmd_d = msg->data.data[3] * 100;
  ros_cmd_e = msg->data.data[4] * 100;
  ros_cmd_f = msg->data.data[5] * 100;
  ros_cmd_g = msg->data.data[6] * 100;
  ros_cmd_h = msg->data.data[7] * 100;
  delay(20);
  digitalWrite(led, LOW);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  // Serial.println("start");
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&depth_pub, &msg_depth, NULL));
    if (depth_sensor_init == true) {
      msg_depth.data = depth_sensor.depth();
    } else {
      msg_depth.data = -1.0;
      if (depth_sensor.init(Wire2)) {
        depth_sensor_init = true;
        init_depth_sensor();
        depth_sensor.read();
        msg_depth.data = depth_sensor.depth();
      }
    }
    // digitalWrite(led, !digitalRead(led));
  }
  // digitalWrite(led, !digitalRead(led));
  // Serial.println("Timer callback");
}

bool ros_create_entities() {
  // Initialize micro-ROS allocator
  delay(1000);
  Wire2.begin();
  allocator = rcl_get_default_allocator();

  
  
  msg_in.data.capacity = 8; 
  msg_in.data.size = 0;
  msg_in.data.data = (float_t*) malloc(msg_in.data.capacity * sizeof(float_t));
  

  msg_in.layout.dim.capacity = 8;
  msg_in.layout.dim.size = 0;
  msg_in.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_in.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for(size_t i = 0; i < msg_in.layout.dim.capacity; i++){
      msg_in.layout.dim.data[i].label.capacity = 8;
      msg_in.layout.dim.data[i].label.size = 0;
      msg_in.layout.dim.data[i].label.data = (char*) malloc(msg_in.layout.dim.data[i].label.capacity * sizeof(char));
  }



  //create init_options
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  // node_ops.domain_id = (size_t)(12);
  size_t domain_id = (size_t)(12);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  // RCCHECK(rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support, &node_ops));

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	// create node
	RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create thrust subscribers
  RCCHECK(rclc_subscription_init_default(
    &motors_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/thrust_cmds"));

  RCCHECK(rclc_publisher_init_default(
    &depth_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/depth_sensor"));

  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));  // Increment this for more subs

  RCCHECK(rclc_executor_add_subscription(&executor, &motors_sub, &msg_in, &motors_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  if (!depth_sensor.init(Wire2)) {
    digitalWrite(led, !digitalRead(led));
    depth_sensor_init = false;
    msg_depth.data = -1.0;
  }

  // while (!depth_sensor.init(Wire2)) {
  //   digitalWrite(led, !digitalRead(led));
  //   delay(2000);
  // }
  
  if (depth_sensor_init == true) {
    init_depth_sensor();
  }

  return true;
}

void init_depth_sensor() {
  depth_sensor.setModel(MS5837::MS5837_30BA);
  depth_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  msg_depth.data = 0.0;
}

void ros_destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&motors_sub, &node);
  rcl_publisher_fini(&depth_pub, &node);
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
  // create participant, delete client, destroy session, create client, establish session, establish session, establish session, establish session
  // destroy entities, connected, waiting
  // Vehicle Logic
  if (killed) {
    // TODO: Listen for killed on actual E-stop circuit in case of manual shutoff
    // TODO: Add a time delay before resuming from killed state with blink
    // cfg_lt(1, 0, 0, 0);
    zero_ros_cmds();
    zero_all_motors();
  } else {
    if (true) {  // AUTONOMOUS
      ros_handler();
      motor_a.writeMicroseconds(throttleToESC(ros_cmd_a));
      motor_b.writeMicroseconds(throttleToESC(ros_cmd_b));
      motor_c.writeMicroseconds(throttleToESC(ros_cmd_c));
      motor_d.writeMicroseconds(throttleToESC(ros_cmd_d));
      motor_e.writeMicroseconds(throttleToESC(ros_cmd_e));
      motor_f.writeMicroseconds(throttleToESC(ros_cmd_f));
      motor_g.writeMicroseconds(throttleToESC(ros_cmd_g));
      motor_h.writeMicroseconds(throttleToESC(ros_cmd_h));
      delay(20);
    // } else if (mode == 1) {  // CALIBRATION
    //   // calibrate_rc();
    //   cfg_lt(0, 2, 0, 0);
    // } else if (mode == 2) {  // REMOTE CONTROL
    //   // set_motor_6x();
    //   cfg_lt(0, 1, 0, 0);
    //   Serial.println("Throttle set");
    //   motor_a.writeMicroseconds(throttleToESC(rc_cmd_a));
    //   motor_b.writeMicroseconds(throttleToESC(rc_cmd_b));
    //   motor_c.writeMicroseconds(throttleToESC(rc_cmd_c));
    //   motor_d.writeMicroseconds(throttleToESC(rc_cmd_d));
    //   motor_e.writeMicroseconds(throttleToESC(rc_cmd_e));
    //   motor_f.writeMicroseconds(throttleToESC(rc_cmd_f));
    } else {
      Serial.println("Error, mode not supported");
    }
  }
}

void setup() {
  // Serial.begin(9600);
  loop_time = millis();

  delay(100);
  // Serial.println("NOVA MOTOR STARTING...");
  // Serial.println("SETTING UP AND TESTING LIGHT TOWER...");

  // Serial.println(sizeof(float_t));

  // Serial.println("INITIALIZING MOTORS...");
  motor_a.attach(A_SIG_PIN);
  motor_b.attach(B_SIG_PIN);
  motor_c.attach(C_SIG_PIN);
  motor_d.attach(D_SIG_PIN);
  motor_e.attach(E_SIG_PIN);
  motor_f.attach(F_SIG_PIN);
  motor_g.attach(G_SIG_PIN);
  motor_h.attach(H_SIG_PIN);

  motor_a.writeMicroseconds(1500);
  motor_b.writeMicroseconds(1500);
  motor_c.writeMicroseconds(1500);
  motor_d.writeMicroseconds(1500);
  motor_e.writeMicroseconds(1500);
  motor_f.writeMicroseconds(1500);
  motor_g.writeMicroseconds(1500);
  motor_h.writeMicroseconds(1500);
  delay(2500);

  // Serial.println("SETTING UP MICROROS TRANSPORTS...");
  set_microros_transports();

  state = WAITING_AGENT;

  // Serial.println("==================================================");
  // Serial.println("============ NOVA MOTOR INIT COMPLETE ============");
  // Serial.println("==================================================");
  pinMode(led, OUTPUT);
}

void zero_all_motors() {
  motor_a.writeMicroseconds(1500);
  motor_b.writeMicroseconds(1500);
  motor_c.writeMicroseconds(1500);
  motor_d.writeMicroseconds(1500);
  motor_e.writeMicroseconds(1500);
  motor_f.writeMicroseconds(1500);
  motor_g.writeMicroseconds(1500);
  motor_h.writeMicroseconds(1500);
}
void zero_ros_cmds() {
  ros_cmd_a = 0;
  ros_cmd_b = 0;
  ros_cmd_c = 0;
  ros_cmd_d = 0;
  ros_cmd_e = 0;
  ros_cmd_f = 0;
  ros_cmd_g = 0;
  ros_cmd_h = 0;
}


void ros_handler() {
  bool created = false;
  switch (state) {
    case WAITING_AGENT:
      zero_ros_cmds();
      zero_all_motors();
      EXECUTE_EVERY_N_MS(1500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 2)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
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
      EXECUTE_EVERY_N_MS(7500, state = (RMW_RET_OK == rmw_uros_ping_agent(1500, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      break;
    case AGENT_DISCONNECTED:  
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
int cmd_ctr = 0;

void loop() {
  // Get loop time
  loop_time = millis();

  if (depth_sensor_init == true) {
    depth_sensor.read();
  } else {
    if (depth_sensor.init(Wire2)) {
      depth_sensor_init = true;
      init_depth_sensor();
      depth_sensor.read();
    }
  }
  
  // E-Stop
  //  read_estop();
  // Polling R/C commands
  // read_rc();
  // Execute based on mode
  boat_killed = false;
  cmd_ctr = 0;
  exec_mode(cmd_ctr, boat_killed);
  // run_lt(lt_red_state, lt_yel_state, lt_grn_state, lt_blu_state);
  if (state == AGENT_CONNECTED) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
}
