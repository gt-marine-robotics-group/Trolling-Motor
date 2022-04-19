#include <LapX9C10X.h>

//#include <micro_ros_arduino.h>

/*
  GITMRG Nova Custom Trolling Motor Driver

  Interfaces with PWM Module through digital potentiometer
  PWM Module is interfaced with NV-series 8-speed trolling motor.

  v0.1
  modified 20 FEB 2022
  by Sean Fish, Alvaro Pelaez

  Roadmap
   - Version 0: Arduino Nano - Serial Commuication
   - Version 1: Arduino Due - micro-ROS

  Resources
  https://stackoverflow.com/questions/6504211/is-it-possible-to-include-a-library-from-another-library-using-the-arduino-ide
*/

class Motor {
  private:
    int dirOnePin;
    int dirTwoPin;
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
      throttle = new LapX9C10X(throIncPin, throUdPin, throCsPin, throRes);
      init();
    }
    void init() {
      throttle->begin(-1); // Min resistance
      pinMode(dirOnePin, OUTPUT);
      pinMode(dirTwoPin, OUTPUT);
    }
    bool setThrottle(int throttleValue) {
      setDirection(throttleValue);
      throttle->set(abs(throttleValue));
      return true;
    }
};

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
//const int B_THRO_CS_PIN = 28;
//const int B_THRO_UD_PIN = 29;
//const int B_THRO_INC_PIN = 30;
//const int B_DIR_SEL0_PIN = 32;
//const int B_DIR_SEL1_PIN = 33;
// MOTOR CHARLIE

// MOTOR DELTA


// LIGHT TOWER
const int LT_RED_PIN;
const int LT_YEL_PIN;
const int LT_GRN_PIN;
const int LT_BLU_PIN;

// VARS -------------------------------------------------------------
const int THRO_RESISTANCE = LAPX9C10X_X9C104;
const int throttleMax = 50;
int m_signal = 0;
bool debug = true;
int control_state = 1; // 0 - KILLED | 1 - STANDBY | 2 - MANUAL | 3 - AUTONOMOUS | 4 - AUXILIARY
// add a blinking delay for restoring power after kill state


// DEVICES ----------------------------------------------------------
// MOTOR ALFA
Motor motor_a(A_THRO_INC_PIN, A_THRO_UD_PIN, A_THRO_CS_PIN, THRO_RESISTANCE, A_DIR_SEL0_PIN, A_DIR_SEL1_PIN);

// FUNCTIONS --------------------------------------------------------
void subscription_callback(const void * msgin) {
  Serial.println("CALLBACK");
}

// Need function for translating RC to motor
// Need publish vehicle state

void setup() {
  Serial.begin(9600);
  Serial.println("NOVA MOTOR STARTING...");
  
  delay(5000);
  debug = false;

  Serial.println("==================================================");
  Serial.println("============ NOVA MOTOR INIT COMPLETE ============");
  Serial.println("==================================================");
}


void loop() {
  // Demo test loop
  int counter;
  //  direction=2;   //BWD
  //  set_direction(direction);

  for (counter = -1 * throttleMax; counter < throttleMax; counter++) {
    Serial.print("Inc: counter = ");
    Serial.print(counter);
    motor_a.setThrottle(counter);
    delay(100);
  }
  for (counter = throttleMax - 1; counter >= -1 * throttleMax; counter--) {
    Serial.print("Decc: counter = ");
    Serial.print(counter);
    motor_a.setThrottle(counter);
    delay(100);
  }
}
