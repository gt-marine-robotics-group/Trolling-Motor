#include <LapX9C10X.h>

#include <micro_ros_arduino.h>

/*
  GITMRG Nova Custom Trolling Motor Driver

  Interfaces with PWM Module through digital potentiometer
  PWM Module is interfaced with NV-series 8-speed trolling motor.  

  v0.1
  modified 20 FEB 2022
  by Sean Fish, Alvaro Pelaez

  Roadmap
   - Version 0: Arduino Nano - Serial Commuication
   - Version 1: Raspberry Pi Pico - micro-ROS

  Resources
  https://stackoverflow.com/questions/6504211/is-it-possible-to-include-a-library-from-another-library-using-the-arduino-ide
*/

// PINS -------------------------------------------------------------
// RC INPUT
const int ORX_AUX1_PIN = 1; // checks if killed - need to figure out reset check
const int ORX_GEAR_PIN = 2; // Auto vs Manual
const int ORX_RUDD_PIN = 3; // yaw
const int ORX_ELEV_PIN = 4; // WAM-V translate forward / backward
const int ORX_AILE_PIN = 5; // WAM-V translate left / right
const int ORX_THRO_PIN = 6;

// MOTOR ALFA
const int A_THRO_CS_PIN = 22;
const int A_THRO_UD_PIN = 23;
const int A_THRO_INC_PIN = 24;
const int A_DIR_SEL0_PIN = 26;
const int A_DIR_SEL1_PIN = 27;
// MOTOR BRAVO
const int B_THRO_CS_PIN = 28;
const int B_THRO_UD_PIN = 29;
const int B_THRO_INC_PIN = 30;
const int B_DIR_SEL0_PIN = 32;
const int B_DIR_SEL1_PIN = 33;
// MOTOR CHARLIE

// MOTOR DELTA


// LIGHT TOWER


// VARS -------------------------------------------------------------
const int THRO_RESISTANCE = LAPX9C10X_X9C104;
const int throttleMax = 50;
int m_signal=0;
bool debug = true;
int control_state = 1; // 0 - KILLED | 1 - STANDBY | 2 - MANUAL | 3 - AUTONOMOUS | 4 - AUXILIARY
// add a blinking delay for restoring power after kill state 


// DEVICES ----------------------------------------------------------
// MOTOR ALFA
LapX9C10X a_throttle(A_THRO_INC_PIN, A_THRO_UD_PIN, A_THRO_CS_PIN, THRO_RESISTANCE);
int a_direction = 0; // 0 - OFF | 1 - FWD | 2 - BWD
// MOTOR BRAVO
LapX9C10X b_throttle(B_THRO_INC_PIN, B_THRO_UD_PIN, B_THRO_CS_PIN, THRO_RESISTANCE);
int b_direction = 0;
// MOTOR CHARLIE

// MOTOR DELTA

// FUNCTIONS --------------------------------------------------------
void subscription_callback(const void * msgin) {
  Serial.println("CALLBACK");
}

// Need function for translating RC to motor
// Need publish vehicle state

void setup() {
  Serial.begin(9600);
  pinMode(A_DIR_SEL0_PIN, OUTPUT);
  pinMode(A_DIR_SEL1_PIN, OUTPUT);
  Serial.println("NOVA MOTOR STARTING...");
  throttle.begin(-1); // Min resistance
  delay(5000);
  debug = false;

  Serial.println("==================================================");
  Serial.println("============ NOVA MOTOR INIT COMPLETE ============");
  Serial.println("==================================================");
}

void set_direction(int d) {
  if (d==0){ //OFF
    digitalWrite(DIR_SEL0_PIN, LOW);
    digitalWrite(DIR_SEL1_PIN, LOW);
    Serial.println("DIRECTION OFF");
  }
  else if (d==1){ //FWD
    digitalWrite(DIR_SEL0_PIN, LOW);
    digitalWrite(DIR_SEL1_PIN, HIGH);
    Serial.println("DIRECTION 1");
  }
  else if (d==2){ //BWD
    digitalWrite(DIR_SEL0_PIN, HIGH);
    digitalWrite(DIR_SEL1_PIN, LOW);
    Serial.println("DIRECTION 2");
  }
}

void loop() {
  // Demo test loop
  int counter;
//  direction=2;   //BWD
//  set_direction(direction);
  
  for(counter = 0; counter < throttleMax; counter++) {
    Serial.print("Inc: counter = ");
    Serial.print(counter);
    throttle.set(counter);
    Serial.print(", new resistance = ");
    Serial.print(throttle.getK());
    Serial.println("KOhms");
    delay(100);
  }
  for(counter = throttleMax - 1; counter >= 0; counter--) {
    Serial.print("Decc: counter = ");
    Serial.print(counter);
    throttle.set(counter);
    Serial.print(", new resistance = ");
    Serial.print(throttle.getK());
    Serial.println("KOhms");
    delay(100);
  }

  // Check signal for motor command
  
 //Set motor to command value
  if (m_signal<0) {
    direction=2;   //BWD
    set_direction(direction);
    //throttle.set(abs(m_signal));
  }
  else if (m_signal==0) {
    direction=0;   //OFF
    set_direction(direction);
    //throttle.set(0);
  }
  else {
    direction=1;   //FWD
    set_direction(direction);
    //throttle.set(m_signal);
  }
  m_signal = ((m_signal + 1 + 1) % 3) - 1;

}
