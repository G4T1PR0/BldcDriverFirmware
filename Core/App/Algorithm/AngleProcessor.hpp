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
        _polePairs = 6;
        _direction = 0;
    }

    void update() {
        _encoder->update();
        _mechanicalAngle = _encoder->getAngle();
        _electricalAngle = _calcElectricalAngle();
        _velocity = _encoder->getVelocity();
    }

    float getMechanicalAngle() {
        return _mechanicalAngle;
    }

    float getElectricalAngle() {
        return _electricalAngle;
    }

    float getVelocity() {
        return _velocity;
    }

    void setPolePairs(uint16_t polePairs) {
        _polePairs = polePairs;
    }

    void setDirection(bool direction) {
        _direction = direction;
    }

    void setZero() {
        _zero_electrical_angle = _calcElectricalAngle();
    }

   private:
    baseEncoder* _encoder;

    uint16_t _polePairs = 1;
    bool _direction = 1;

    float _mechanicalAngle = 0;
    float _electricalAngle = 0;
    float _zero_electrical_angle = 0;

    float _prev_mechanicalAngle = 0;
    float _velocity = 0;

    float _calcElectricalAngle() {
        if (_direction) {
            return _normalizeAngle(-1 * _polePairs * _mechanicalAngle - _zero_electrical_angle);
        } else {
            return _normalizeAngle(_polePairs * _mechanicalAngle - _zero_electrical_angle);
        }
    }

    float _normalizeAngle(float angle) {
        float a = fmod(angle, 2 * M_PI);

        if (a < 0) {
            a += 2 * M_PI;
        }

        return a;
    }
};
