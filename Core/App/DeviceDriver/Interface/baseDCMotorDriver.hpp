/*
 * baseDCMotorDriver.hpp
 *
 *  Created on: Nov 8, 2024
 *      Author: G4T1PR0
 */

#pragma once

class baseDCMotorDriver {
   public:
    virtual void init(void) = 0;
    virtual void update(void) = 0;
    virtual void setPWM(float duty) = 0;
    virtual void enable(bool) = 0;
};
