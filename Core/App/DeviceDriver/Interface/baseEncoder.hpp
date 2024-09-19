/*
 * baseEncoder.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once
#include <stdint.h>

class baseEncoder {
   public:
    virtual void init() = 0;
    virtual void update() = 0;
    virtual int32_t getCnt() = 0;
    virtual int32_t getTotalCnt() = 0;
    virtual float getAngle() = 0;
    virtual float getVelocity() = 0;
    virtual void setZero() = 0;
};