#include "light_tower.h"

#include <Arduino.h>

void LightTower::setup() {
    for (int pin : m_light_pins) {
        pinMode(pin, OUTPUT);
    }
}

void LightTower::display() {
    uint32_t loop_time = millis();
    for (int i {0}; i < m_lights.size(); ++i) {
        LightStates state = m_lights[i];
        bool light_on;
        if (state == LightStates::off) {
            light_on = false;
        }
        else if (state == LightStates::on) {
            light_on = true;
        }
        else if (state == LightStates::flashing) {
            if (loop_time % 500 <= 200) {
            light_on = true;
            }
        }
        else if (state == LightStates::fast_flashing) {
            if (loop_time % 1000 <= 400) {
            light_on = true;
            }
        }
        digitalWrite(m_light_pins[i], light_on);
    }
}

void LightTower::configure(LightStates red, LightStates yellow, LightStates green, LightStates blue) {
    m_lights[Colors::red] = red;
    m_lights[Colors::yellow] = yellow;
    m_lights[Colors::green] = green;
    m_light_pins[Colors::blue] = blue;
}