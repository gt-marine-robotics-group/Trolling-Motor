#include "remote_control.h"
#include <Arduino.h>
#include <ServoInput.h>
#include <cstdlib>

void RemoteControl::center_rc() {
  RemoteControl::center_pin(m_orx_elev);
  RemoteControl::center_pin(m_orx_aile);
  RemoteControl::center_pin(m_orx_rudd);
}

void RemoteControl::read() {
    m_srg = m_orx_elev.mapDeadzone(-100, 101, 0.05);
    m_swy = m_orx_aile.mapDeadzone(-100, 101, 0.05);
    m_yaw = m_orx_rudd.mapDeadzone(-100, 101, 0.05);
    m_ctr_state = static_cast<ControlState>(m_orx_aux1.map(0, 2));
    // m_kill_state = static_cast<KillState>(m_orx_gear.map(1, 0)); # WRONG ?
    char buffer[100];
    sprintf(buffer, "RC | SRG: %4i  SWY: %4i  YAW: %4i CTR: %1i KIL: %1i", 
        m_srg, m_swy, m_yaw, m_ctr_state, m_kill_state);
    Serial.println(buffer);
}

bool RemoteControl::check_calibration_ready() {
    bool e_s = RemoteControl::check_pin_calibrated(m_orx_elev);
    bool a_s = RemoteControl::check_pin_calibrated(m_orx_aile);
    bool r_s = RemoteControl::check_pin_calibrated(m_orx_rudd);
    char buffer[100];
    sprintf(buffer, "Calibrated Values | Elev: %4i  Aile: %4i  Rudd: %4i", 
        m_orx_elev.mapDeadzone(-100, 100, 0.1), 
        m_orx_aile.mapDeadzone(-100, 100, 0.1), 
        m_orx_rudd.mapDeadzone(-100, 100, 0.1)
    );
    Serial.println(buffer);
    return (e_s && a_s && r_s);
}

void RemoteControl::calibrate() {
    center_rc();

    uint32_t loop_time = millis();

    bool calibration_ready = false;
    int calibration_zero_check = 0;

    while(m_ctr_state != ControlState::calibration || !calibration_ready || !(calibration_zero_check < 15)) {
        loop_time = millis();
        // run_lt(2, 2, 0, 0);
        read();
        calibration_ready = check_calibration_ready();
        if (abs(m_srg) + abs(m_swy) + abs(m_yaw) <= 4) {
        calibration_zero_check += 1;
        }
        else {
        calibration_zero_check = 0;
        }
    }
}

int RemoteControl::get_srg() const {
    return m_srg;
}

int RemoteControl::get_swy() const {
    return m_swy;
}

int RemoteControl::get_yaw() const {
    return m_yaw;
}

int RemoteControl::get_ctr_state() const {
    return m_ctr_state;
}