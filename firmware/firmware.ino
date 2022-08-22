#include <LapX9C10X.h>
#include <ServoInput.h>

//#include <micro_ros_arduino.h>
//#include <stdio.h>
//#include <rcl/rcl.h>
//#include <rcl/error_handling.h>
//#include <rclc/rclc.h>
//#include <rclc/executor.h>
//#include <std_msgs/msg/int32.h>

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
*/

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
    bool setThrottle(int throttleValue) {
      setDirection(throttleValue);
      throttle->reset(abs(throttleValue));
      return true;
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

int cmd_srg; // Commanded Surge
int cmd_swy; // Commanded Sway
int cmd_yaw; // Commanded Yaw
int cmd_ctr; // Commanded Control State
int cmd_kil; // Commanded Kill State

int cmd_a;
int cmd_b;
int cmd_c;
int cmd_d;

// FUNCTIONS --------------------------------------------------------
void subscription_callback(const void * msgin) {
  Serial.println("CALLBACK");
}

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
  cmd_a = a / max_val;
  cmd_b = b / max_val;
  cmd_c = c / max_val;
  cmd_d = d / max_val;
}

// Translate RC input to 2 motor system
void set_motor_2x() {
  int a = (cmd_srg - cmd_yaw);
  int d = (cmd_srg + cmd_yaw);
  float max_val = max(100, max(abs(a), abs(d))) / 100;
  cmd_a = a / max_val;
  cmd_b = 0;
  cmd_c = 0;
  cmd_d = d / max_val;
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

void exec_mode(int mode, bool killed) {
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
    if (mode == 0){
    // Autonomous
    Serial.println("Autonomous");
    run_lt(0, 0, 1, 0);
    motor_a.setThrottle(0);
    motor_b.setThrottle(0);
    motor_c.setThrottle(0);
    motor_d.setThrottle(0);
    }
    else if (mode == 1) {
      // Calibrate
      calibrate_rc();
      run_lt(0, 2, 0, 0);
      motor_a.setThrottle(0);
      motor_b.setThrottle(0);
      motor_c.setThrottle(0);
      motor_d.setThrottle(0);
    }
    else if (mode == 2) {
      set_motor_4x();
      run_lt(0, 1, 0, 0);
      char buffer[100];
       sprintf(buffer, "Manual | A: %4i  B: %4i  C: %4i D: %4i", 
      cmd_a, cmd_b, cmd_c, cmd_d);
      Serial.println(buffer);
      motor_a.setThrottle(cmd_a);
      motor_b.setThrottle(cmd_b);
      motor_c.setThrottle(cmd_c);
      motor_d.setThrottle(cmd_d);
    }
    else {
      Serial.println("Error, mode not supported");
    }
  }
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
//rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
//rclc_executor_t executor;
//rclc_support_t support;
//rcl_allocator_t allocator;
//rcl_node_t node;
//rcl_timer_t timer;

void setup() {
  Serial.begin(9600);
  loop_time = millis();
  //set_microros_transports();
  //delay(2000);
  //allocator = rcl_get_default_allocator();
  // Create init_options
  //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // Create node
  //RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  // Create publisher
  //RCCHECK(rclc_publisher_init_default(
  //  &publisher,
  //  &node,
  //  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //  "micro_ros_arduino_node_publisher"));
  // create subscriber
  //RCCHECK(rclc_subscription_init_default(
  //  &subscriber,
  //  &node,
  //  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //  "micro_ros_arduino_subscriber"));
  // Create executor
  //RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  //RCCHECK(rclc_executor_add_timer(&executor, &timer));
  //RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
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


void loop() {
  // Get loop time
  loop_time = millis();
  // Polling R/C commands
  read_rc();
  // Execute based on mode
  // Serial.println("execute");
  exec_mode(cmd_ctr, cmd_kil);
  //RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
