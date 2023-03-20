#include "motors_6.h"
#include "remote_control.h"
#include "light_tower.h"
#include "micro_ros.h"

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
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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

MicroRos ros {6};

int estop_state; // ESTOP HIGH or LOW (LOW = KILLED)

// FUNCTIONS --------------------------------------------------------

// namespace std {
// void __throw_bad_alloc() {
//   Serial.println("Unable to allocate memory");
// }
// void __throw_length_error( char const*e ) {
//   Serial.print("Length Error :"); Serial.println(e);
// }
// }

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

LightTower lt{};

void exec_mode(int mode, bool killed) {
  
  // Vehicle Logic
  if (killed) {
    // TODO: Listen for killed on actual E-stop circuit in case of manual shutoff
    // TODO: Add a time delay before resuming from killed state with blink
    // cfg_lt(1, 0, 0, 0);
    lt.configure(LightTower::LightStates::on, LightTower::LightStates::off,
      LightTower::LightStates::off, LightTower::LightStates::off);
  }
  else {
    if (mode + 1 == RemoteControl::ControlState::autonomous){ // AUTONOMOUS
      ros.execute(lt, motors);
      delay(20);
    }
    else if (mode + 1 == RemoteControl::ControlState::calibration) { // CALIBRATION
      rc.check_calibration_ready();
      lt.configure(LightTower::LightStates::off, LightTower::LightStates::flashing,
        LightTower::LightStates::off, LightTower::LightStates::off);
    }
    else if (mode + 1 == RemoteControl::ControlState::remote_control) { // REMOTE CONTROL
      // set_motor_6x();
      std::vector<int> cmds = motors.get_rc_motor_cmds(rc);
      // cfg_lt(0, 1, 0, 0);
      lt.configure(LightTower::LightStates::off, LightTower::LightStates::on,
        LightTower::LightStates::off, LightTower::LightStates::off);
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
  
  delay(100);
  Serial.println("NOVA MOTOR STARTING...");
  Serial.println("SETTING UP LIGHT TOWER...");
//  setup_estop();
  lt.setup();

  Serial.println("INITIALIZING MOTOR CONTROLLERS...");

  Serial.println("CALIBRATING CONTROLLER...");
  rc.calibrate(lt);

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

// TODO: Refactor to use control_state
bool boat_killed = false;

void loop() {
  // Get loop time
  loop_time = millis();
  // E-Stop
//  read_estop();
  // Polling R/C commands
  rc.read();
  // Execute based on mode
  boat_killed = false;
  exec_mode(rc.get_ctr_state(), boat_killed);
  // Update light tower
  if (estop_state == true) {
    lt.configure(LightTower::LightStates::off, LightTower::LightStates::off,
      LightTower::LightStates::on, LightTower::LightStates::on);
  }
  lt.display();
  ros.spin();
}
