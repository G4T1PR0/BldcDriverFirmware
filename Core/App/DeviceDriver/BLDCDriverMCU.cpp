/*
 * BLDCDriverMCU.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#include <DeviceDriver/BLDCDriverMCU.hpp>

BLDCDriverMCU::BLDCDriverMCU(MAL* mcu, MAL::P_PWM u, MAL::P_PWM v, MAL::P_PWM w) {
    _mcu = mcu;
    _u = u;
    _v = v;
    _w = w;
}

void BLDCDriverMCU::init(void) {
    _enabled = false;
}

void BLDCDriverMCU::update(void) {
    if (_enabled) {
        if (_u_duty != _prev_u_duty) {
            _mcu->pwmSetDuty(_u, _u_duty);
            _prev_u_duty = _u_duty;
        }

        if (_v_duty != _prev_v_duty) {
            _mcu->pwmSetDuty(_v, _v_duty);
            _prev_v_duty = _v_duty;
        }

        if (_w_duty != _prev_w_duty) {
            _mcu->pwmSetDuty(_w, _w_duty);
            _prev_w_duty = _w_duty;
        }
    } else {
        _mcu->pwmSetDuty(_u, 0);
        _mcu->pwmSetDuty(_v, 0);
        _mcu->pwmSetDuty(_w, 0);
    }
}

void BLDCDriverMCU::setPWM(float u, float v, float w) {
    if (u > 1) {
        _u_duty = 1;
    } else if (u < 0) {
        _u_duty = 0;
    } else {
        _u_duty = u;
    }

    if (v > 1) {
        _v_duty = 1;
    } else if (v < 0) {
        _v_duty = 0;
    } else {
        _v_duty = v;
    }

    if (w > 1) {
        _w_duty = 1;
    } else if (w < 0) {
        _w_duty = 0;
    } else {
        _w_duty = w;
    }
}

void BLDCDriverMCU::enable(bool enable) {
    _enabled = enable;
}
