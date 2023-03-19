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

        void write_from_rc(const std::vector<int>& commands);

        void write_from_ros(const std::vector<int>& commands);
};

#endif