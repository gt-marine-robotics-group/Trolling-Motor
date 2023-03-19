#ifndef LIGHT_TOWER_H
#define LIGHT_TOWER_H

#include <array>

namespace light_tower_constants {
    constexpr int LT_RED_PIN = 58; // A4
    constexpr int LT_GRN_PIN = 57; // A3
    constexpr int LT_YEL_PIN = 59; // A5
    constexpr int LT_BLU_PIN = 60; // A6
};

class LightTower {

    public:

        enum LightStates {
            off, on, flashing, fast_flashing
        };

        enum Colors {
            red, yellow, green, blue
        };
    
    private:

        std::array<LightStates, 4> m_lights {{LightStates::off}};
        std::array<int, 4> m_light_pins {{light_tower_constants::LT_RED_PIN, 
            light_tower_constants::LT_YEL_PIN, light_tower_constants::LT_GRN_PIN,
            light_tower_constants::LT_BLU_PIN}};
    
    public:

        void setup();

        void display();

        void configure(LightStates red, LightStates yellow, LightStates green, LightStates blue);
};

#endif