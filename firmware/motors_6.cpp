#include <motors_6.h>
#include <motors.h>

#include <vector>

const std::vector<int> Motor6::default_pins {8, 9, 10, 11, 12, 7};

Motor6::Motor6() : Motors{6} {};
