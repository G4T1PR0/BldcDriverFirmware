/*
 * DCMotorDriverMCU.hpp
 *
 *  Created on: Nov 8, 2024
 *      Author: G4T1PR0
 */

#include <DeviceDriver/DCMotorDriver.hpp>

DCMotorDriverMCU::DCMotorDriverMCU(MAL* mcu, MAL::P_PWM hb1, MAL::P_PWM hb2) {
    _mcu = mcu;
    _hb1 = hb1;
    _hb2 = hb2;
}

void DCMotorDriverMCU::init(void) {
    _enabled = false;
}

void DCMotorDriverMCU::update(void) {
    if (_enabled) {
        if (_hb1_duty != _prev_hb1_duty) {
            _mcu->pwmSetDuty(_hb1, _hb1_duty);
            _prev_hb1_duty = _hb1_duty;
        }
        if (_hb2_duty != _prev_hb2_duty) {
            _mcu->pwmSetDuty(_hb2, _hb2_duty);
            _prev_hb2_duty = _hb2_duty;
        }
    } else {
        _mcu->pwmSetDuty(_hb1, 0);
        _mcu->pwmSetDuty(_hb2, 0);
    }
}

void DCMotorDriverMCU::setPWM(float duty) {
    if (duty > 0) {
        _hb1_duty = duty;
        _hb2_duty = 0;
    } else if (duty < 0) {
        _hb1_duty = 0;
        _hb2_duty = duty;
    } else {
        _hb1_duty = 0;
        _hb2_duty = 0;
    }
}

void DCMotorDriverMCU::enable(bool en) {
    _enabled = en;
}