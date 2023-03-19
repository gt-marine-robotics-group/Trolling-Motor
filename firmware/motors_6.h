#ifndef MOTOR_6_H
#define MOTOR_6_H

#include "motors.h"
#include "remote_control.h"
#include <Servo.h>
#include <vector>

class Motor6 : public Motors {

    enum ServoNames {
        back_left, middle_left, front_left, front_right, middle_right, back_right
    };

    public:

        static const std::vector<int> default_pins;

        Motor6();

        const std::vector<int> get_rc_motor_cmds(const RemoteControl& rc);
};

#endif