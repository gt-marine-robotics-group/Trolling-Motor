
// LIGHT TOWER VARS AND FUNCTIONS
// TODO: Split into separate library

// PINS -------------------------------------------------------------
const int LT_RED_PIN = A4;
const int LT_GRN_PIN = A3;
const int LT_YEL_PIN = A5;
const int LT_BLU_PIN = A6;
int lt_red_state = 0;
int lt_grn_state = 0;
int lt_yel_state = 0;
int lt_blu_state = 0;

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