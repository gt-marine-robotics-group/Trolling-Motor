#include <LapX9C10X.h>
#include <ServoInput.h>

//#include <micro_ros_arduino.h>

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

    /*
    bool addThrottle(int throttleValue) {
      int num = (throttleValue + throttle->get(throttleValue)) % 100;
      setDirection(num);
      throttle->set(abs(num));
      return truel
    }
    */
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
const int A_THRO_CS_PIN = 23;
const int A_THRO_UD_PIN = 25;
const int A_THRO_INC_PIN = 27;
const int A_DIR_SEL0_PIN = 29;
const int A_DIR_SEL1_PIN = 31;
// MOTOR BRAVO
const int B_THRO_CS_PIN = 14;
const int B_THRO_UD_PIN = 15;
const int B_THRO_INC_PIN = 16;
const int B_DIR_SEL0_PIN = 17;
const int B_DIR_SEL1_PIN = 18;
// MOTOR CHARLIE
const int C_THRO_CS_PIN = 7; //Previously Commented
const int C_THRO_UD_PIN = 6;
const int C_THRO_INC_PIN = 5;
const int C_DIR_SEL0_PIN = 4;
const int C_DIR_SEL1_PIN = 3;
// MOTOR DELTA
const int D_THRO_CS_PIN = 12;
const int D_THRO_UD_PIN = 11;
const int D_THRO_INC_PIN = 10;
const int D_DIR_SEL0_PIN = 9;
const int D_DIR_SEL1_PIN = 8;

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


// DEVICES ----------------------------------------------------------
// MOTORS
//Motor motor_a(A_THRO_INC_PIN, A_THRO_UD_PIN, A_THRO_CS_PIN, THRO_RESISTANCE, A_DIR_SEL0_PIN, A_DIR_SEL1_PIN);
//Motor motor_b(B_THRO_INC_PIN, B_THRO_UD_PIN, B_THRO_CS_PIN, THRO_RESISTANCE, B_DIR_SEL0_PIN, B_DIR_SEL1_PIN);
//Motor motor_c(C_THRO_INC_PIN, C_THRO_UD_PIN, C_THRO_CS_PIN, THRO_RESISTANCE, C_DIR_SEL0_PIN, C_DIR_SEL1_PIN);
//Motor motor_d(D_THRO_INC_PIN, D_THRO_UD_PIN, D_THRO_CS_PIN, THRO_RESISTANCE, D_DIR_SEL0_PIN, D_DIR_SEL1_PIN);

// RC INPUTS
ServoInputPin<ORX_AUX1_PIN> orxAux1; // 3 states
ServoInputPin<ORX_GEAR_PIN> orxGear; // 2 states
ServoInputPin<ORX_RUDD_PIN> orxRudd; // Continuous
ServoInputPin<ORX_ELEV_PIN> orxElev; // Continuous
ServoInputPin<ORX_AILE_PIN> orxAile; // Continuous
ServoInputPin<ORX_THRO_PIN> orxThro; // Continuous




// FUNCTIONS --------------------------------------------------------
void subscription_callback(const void * msgin) {
  Serial.println("CALLBACK");
}

// Calibrate controller function

// Check current RC status (in order to minimize time polling)

// Translate RC input to 4x holonomic motor system

// Translate RC input to 2 motor system

// Need publish vehicle state

void set_light_tower(int r, int y, int g, int b) {
  // -1 = off | 0 = on | else frequency in hz
  Serial.println("ToDo");
}


void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("NOVA MOTOR STARTING...");
  
  delay(500);
  Serial.println("R...");
  delay(500);
  pinMode(LT_RED_PIN, OUTPUT);
  Serial.println("Y...");
  pinMode(LT_YEL_PIN, OUTPUT);
  Serial.println("G...");
  pinMode(LT_GRN_PIN, OUTPUT);
  Serial.println("B...");
  pinMode(LT_BLU_PIN, OUTPUT);


  Serial.println("==================================================");
  Serial.println("============ NOVA MOTOR INIT COMPLETE ============");
  Serial.println("==================================================");

  // Setup light tower

}

/*
  From orcs-comms/firmware/firmware.ino
*/

int vehicleState;
bool killed;
int state;
int xTranslation;
int yTranslation;
int yaw;

/*
  From orcs-comms/firmware/firmware.ino 
*/


void loop() {
  // Demo test loop
  Serial.println("LOOPING...");
  int counter;
  //  direction=2;   //BWD
  //  set_direction(direction);

  // Polling R/C commands
  // TODO: Make this use an interrupt callback that ensures that control state is switched immediately
  //From orcs-comms/firmware/firmware.ino
//  killed = orxGear.getBoolean();
  state = orxAux1.map(0, 3);
  Serial.println(state);
//  xTranslation = orxElev.map(-100, 100);
//  yTranslation = orxAile.map(-100, 100);
//  yaw = orxRudd.map(-100, 100);
  //From orcs-comms/firmware/firmware.ino

  if (state == 2){
    Serial.println("RC");
    digitalWrite(LT_RED_PIN, LOW);
    digitalWrite(LT_YEL_PIN, HIGH);
    digitalWrite(LT_GRN_PIN, LOW);
    digitalWrite(LT_BLU_PIN, LOW);
  } else if (state == 1) {
    Serial.println("PAUSE");
    digitalWrite(LT_RED_PIN, HIGH);
    digitalWrite(LT_YEL_PIN, LOW);
    digitalWrite(LT_GRN_PIN, LOW);
    digitalWrite(LT_BLU_PIN, LOW);
  } else if (state == 0) {
    Serial.println("AUTO");
    digitalWrite(LT_RED_PIN, LOW);
    digitalWrite(LT_YEL_PIN, LOW);
    digitalWrite(LT_GRN_PIN, HIGH);
    digitalWrite(LT_BLU_PIN, LOW);
  } else {
    Serial.println("AUX");
    digitalWrite(LT_RED_PIN, LOW);
    digitalWrite(LT_YEL_PIN, LOW);
    digitalWrite(LT_GRN_PIN, LOW);
    digitalWrite(LT_BLU_PIN, HIGH);
  }
//  delay(200);
//  if (1) {
//    Serial.println("R...");
//    digitalWrite(LT_RED_PIN, HIGH);
//    digitalWrite(LT_YEL_PIN, LOW);
//    digitalWrite(LT_GRN_PIN, LOW);
//    digitalWrite(LT_BLU_PIN, LOW);
//    
//  } //If
//  delay(200);
//  
//  if(1) {
//    Serial.println("Y...");
//    digitalWrite(LT_RED_PIN, LOW);
//    digitalWrite(LT_YEL_PIN, HIGH);
//    digitalWrite(LT_GRN_PIN, LOW);
//    digitalWrite(LT_BLU_PIN, LOW);
//    
//  } //If
//  delay(200);
//  
//  //if (something == LT_YEL_PIN) {
//  if (1) {
//    Serial.println("G...");
//    digitalWrite(LT_RED_PIN, LOW);
//    digitalWrite(LT_YEL_PIN, LOW);
//    digitalWrite(LT_GRN_PIN, HIGH);
//    digitalWrite(LT_BLU_PIN, LOW);
//    
//  } //If
//  delay(200);
//  
//  //if (something == LT_BLU_PIN) {
//  if (1) {
//    Serial.println("B...");
//    digitalWrite(LT_RED_PIN, LOW);
//    digitalWrite(LT_YEL_PIN, LOW);
//    digitalWrite(LT_GRN_PIN, LOW);
//    digitalWrite(LT_BLU_PIN, HIGH);
//    
//  } //If
}
/*
  for (counter = -1 * throttleMax; counter < throttleMax; counter++) {
    Serial.println("Inc: counter = ");
    Serial.println(counter);
    motor_a.setThrottle(counter);
    delay(100);
  }
  for (counter = throttleMax - 1; counter >= -1 * throttleMax; counter--) {
    Serial.println("Decc: counter = ");
    Serial.println(counter);
    motor_a.setThrottle(counter);
    delay(100);
  }
*/

//  if (xTranslation > 0) {
//
//     motor_a.setThrottle(100);
//     motor_b.setThrottle(100);
//     motor_c.setThrottle(100);
//     motor_d.setThrottle(100);
//     Serial.println("Forward Surge");
//  } // Forward Surge
//
//
//  
//  if (xTranslation < 0) {
//
//     motor_a.setThrottle(-100);
//     motor_b.setThrottle(-100);
//     motor_c.setThrottle(-100);
//     motor_d.setThrottle(-100);
//     Serial.println("Backward Surge");
//  } //Backward Surge
//
//
//  
//  if (yTranslation > 0) {
//
//     motor_a.setThrottle(-100);
//     motor_b.setThrottle(-100);
//     motor_c.setThrottle(100);
//     motor_d.setThrottle(100);
//     Serial.println("Forward Sway");
//  } //Forward Sway
//
//
//
//  
//  if (yTranslation < 0) {
//
//     motor_a.setThrottle(100);
//     motor_b.setThrottle(100);
//     motor_c.setThrottle(-100);
//     motor_d.setThrottle(-100);
//     Serial.println("Backward Sway");
//  } //Backward Sway



  


























  
  
 //Loop
