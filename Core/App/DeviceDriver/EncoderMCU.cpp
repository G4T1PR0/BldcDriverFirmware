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
}

void EncoderMCU::update() {
    _cnt = _mcu->encoderGetCnt(_encoder) - _offset;
    _mcu->encoderSetCnt(_encoder, _offset);
}

int32_t EncoderMCU::getCnt() {
    return _cnt;
}