/*
 * EncoderMCU.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#include <DeviceDriver/EncoderMCU.hpp>

EncoderMCU::EncoderMCU(MAL* mcu, MAL::P_Encoder encoder) {
    _mcu = mcu;
    _encoder = encoder;
}

void EncoderMCU::init() {
    _cnt = 0;
    _total_cnt = 0;
    _cpr = 1024;

    _mcu->encoderSetCnt(_encoder, 0);
}

void EncoderMCU::update() {
    _prev_cnt = _current_cnt;
    _current_cnt = _mcu->encoderGetCnt(_encoder);

    if (_current_cnt - _prev_cnt > 32768) {
        _total_cnt += _current_cnt - _prev_cnt - 65536;
    } else if (_current_cnt - _prev_cnt < -32768) {
        _total_cnt += _current_cnt - _prev_cnt + 65536;
    } else {
        _total_cnt += _current_cnt - _prev_cnt;
    }
}

void EncoderMCU::update1kHz() {
    _velocity = (float)(_total_cnt - _vel_prev_total_cnt) / _cpr * 2 * M_PI / 0.001;
    _vel_prev_total_cnt = _total_cnt;
}

int32_t EncoderMCU::getCnt() {
    return _cnt;
}

int32_t EncoderMCU::getTotalCnt() {
    return _total_cnt;
}

float EncoderMCU::getAngle() {
    return (float)_total_cnt / _cpr * 2 * M_PI;
}

float EncoderMCU::getVelocity() {
    return _velocity;
}

void EncoderMCU::setCpr(uint16_t cpr) {
    _cpr = cpr;
}

void EncoderMCU::setZero() {
    _total_cnt = 0;
}