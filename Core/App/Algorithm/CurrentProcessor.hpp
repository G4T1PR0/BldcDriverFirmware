/*
 * CurrentProcessor.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <DeviceDriver/Interface/baseCurrentSensor.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class CurrentProcessor {
   public:
    CurrentProcessor(baseMcuAbstractionLayer* mcu, baseCurrentSensor* currentSensor) {
        _mcu = mcu;
        _currentSensor = currentSensor;
    }

    struct PhaseCurrent_s {
        float u;
        float v;
        float w;
    };

    struct AlphaBetaCurrent_s {
        float alpha;
        float beta;
    };

    struct DQCurrent_s {
        float d;
        float q;
    };

    void init() {
    }

    void update(float electricalAngle) {
        _phase_current.u = _currentSensor->getCurrent(baseCurrentSensor::U_C);
        _phase_current.v = _currentSensor->getCurrent(baseCurrentSensor::V_C);
        _phase_current.w = _currentSensor->getCurrent(baseCurrentSensor::W_C);

        _alpha_beta_current = _clarkeTransform(_phase_current);
        _dq_current = _parkTransform(_alpha_beta_current, electricalAngle);
    }

    PhaseCurrent_s getPhaseCurrent() {
        return _phase_current;
    }

    AlphaBetaCurrent_s getAlphaBetaCurrent() {
        return _alpha_beta_current;
    }

    DQCurrent_s getDCurrent() {
        return _dq_current;
    }

   private:
    baseMcuAbstractionLayer* _mcu;
    baseCurrentSensor* _currentSensor;

    PhaseCurrent_s _phase_current;
    AlphaBetaCurrent_s _alpha_beta_current;
    DQCurrent_s _dq_current;

    const float _1_sqrt3 = 1.0f / sqrt(3);
    const float _2_sqrt3 = 2.0f / sqrt(3);

    AlphaBetaCurrent_s _clarkeTransform(PhaseCurrent_s phaseCurrent) {
        float mid = (1.0f / 3.0f) * (phaseCurrent.u + phaseCurrent.v + phaseCurrent.w);
        float a = phaseCurrent.u - mid;
        float b = phaseCurrent.v - mid;

        AlphaBetaCurrent_s abCurrent;
        abCurrent.alpha = a;
        abCurrent.beta = _1_sqrt3 * a + _2_sqrt3 * b;
        return abCurrent;
    }

    DQCurrent_s _parkTransform(AlphaBetaCurrent_s alphaBetaCurrent, float electricalAngle) {
        float sin;
        float cos;
        _mcu->cordicSinCos(electricalAngle, &sin, &cos);

        DQCurrent_s dqCurrent;

        dqCurrent.d = alphaBetaCurrent.alpha * cos + alphaBetaCurrent.beta * sin;
        dqCurrent.q = alphaBetaCurrent.beta * cos - alphaBetaCurrent.alpha * sin;

        return dqCurrent;
    }
};