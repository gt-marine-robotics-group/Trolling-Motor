#include "motors_6.h"
#include "motors.h"
#include "remote_control.h"

#include <vector>
#include <cstdlib>

const std::vector<int> Motor6::default_pins {8, 9, 10, 11, 12, 7};

Motor6::Motor6() : Motors{6} {};

const std::vector<int> Motor6::get_rc_motor_cmds(const RemoteControl& rc) {

    std::vector<int> cmds(6);

    int srg = rc.get_srg();
    int swy = rc.get_swy();
    int yaw = rc.get_yaw();

    int a = srg - yaw;
    int b = -swy;
    int c = srg - yaw;
    int d = srg + yaw;
    int e = swy;
    int f = srg + yaw;
    
    
    int max_val = static_cast<int>(max(100, 
      max(max(max(abs(a), abs(b)), max(abs(c), abs(d))), max(abs(e), abs(f)))
    ) / 100.0);

    return {a / max_val, b / max_val, c / max_val, d / max_val, e / max_val, f / max_val};
}
