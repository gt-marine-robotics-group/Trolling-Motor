#ifndef MOTORS_H
#define MOTORS_H

#include <Servo.h>
#include <vector>

class Motors {

    std::vector<Servo> m_servos;

    public:

        Motors(int num_motors);

        void motors_setup(const std::vector<int>& signal_pins);

        void zero_all_motors();
};

#endif