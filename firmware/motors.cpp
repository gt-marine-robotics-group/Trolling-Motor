#include "motors.h"

#include <Servo.h>
#include <ServoInput.h>
#include <vector>

Motors::Motors(int num_motors) : m_servos(num_motors) {};

void Motors::motors_setup(const std::vector<int>& signal_pins) {

    for (int i {0}; i < m_servos.size(); ++i) {
        m_servos[i].attach(signal_pins[i]);
    }

    for (int i {0}; i < m_servos.size(); ++i) {
        m_servos[i].writeMicroseconds(1500);
    }
}

void Motors::zero_all_motors() {

    for (Servo& servo : m_servos) {
        servo.writeMicroseconds(1500);
    }
}

void Motors::write_from_rc(const std::vector<int>& commands) {

    for (int i {0}; i < m_servos.size(); ++i) {
        m_servos[i].writeMicroseconds((commands[i] * 4) + 1500);
    }
}

void Motors::write_from_ros(const std::vector<int>& commands) {

    for (int i {0}; i < m_servos.size(); ++i) {
        m_servos[i].writeMicroseconds((commands[i] * 4) + 1500);
    }
}