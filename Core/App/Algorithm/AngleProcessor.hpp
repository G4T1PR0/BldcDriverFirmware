/*
 * AngleProcessor.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <math.h>
#include <DeviceDriver/Interface/baseEncoder.hpp>

class AngleProcessor {
   public:
    AngleProcessor(baseEncoder* encoder) {
        _encoder = encoder;
    }

    void init() {
        _polePairs = 0;
        _direction = 0;
        _encoderResolution = 0;
    }
    void update() {
        _mechanicalAngle = (float)_encoder->getCnt() * 2 * M_PI / _encoderResolution;
        _electricalAngle = _normalizeAngle(_mechanicalAngle * _polePairs * _direction);
    }

    float getMechanicalAngle() {
        return _mechanicalAngle;
    }

    float getElectricalAngle() {
        return _electricalAngle;
    }

    void setPolePairs(uint16_t polePairs) {
        _polePairs = polePairs;
    }
    void setDirection(bool direction) {
        _direction = direction;
    }
    void setEncoderResolution(uint16_t encoderResolution) {
        _encoderResolution = encoderResolution;
    }

   private:
    baseEncoder* _encoder;

    uint16_t _polePairs;
    bool _direction;
    uint16_t _encoderResolution;

    float _mechanicalAngle;
    float _electricalAngle;

    float _normalizeAngle(float angle) {
        float a = fmod(angle, 2 * M_PI);
        if (a < 0) {
            a += 2 * M_PI;
        }
        return a;
    }
};
