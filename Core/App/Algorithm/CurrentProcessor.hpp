/*
 * CurrentProcessor.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <DeviceDriver/Interface/baseCurrentSensor.hpp>

class CurrentProcessor {
   public:
    CurrentProcessor(baseCurrentSensor* currentSensor) {
        _currentSensor = currentSensor;
    }

    struct PhaseCurrent_t {
        float u;
        float v;
        float w;
    };

    struct AlphaBetaCurrent_t {
        float alpha;
        float beta;
    };

    void init() {
    }
    void update() {
    }

    PhaseCurrent_t getPhaseCurrent() {
    }

    AlphaBetaCurrent_t getAlphaBetaCurrent() {
    }

   private:
    baseCurrentSensor* _currentSensor;

    float _current;
};