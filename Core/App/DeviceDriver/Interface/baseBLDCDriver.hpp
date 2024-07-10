/*
 * baseBLDCDriver.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

class baseBLDCDriver {
   public:
    virtual void init(void) = 0;
    virtual void update(void) = 0;
    virtual void setPWM(float u, float v, float w) = 0;
    virtual void enable(bool) = 0;
};
