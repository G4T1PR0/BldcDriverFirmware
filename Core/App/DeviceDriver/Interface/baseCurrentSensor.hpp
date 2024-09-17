/*
 * baseCurrentSensor.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

class baseCurrentSensor {
   public:
    virtual void init(void) = 0;
    virtual void update(void) = 0;

    enum Current_Sensor {
        U_C,
        V_C,
        W_C,
        End_C,
    };

    virtual float getCurrent(Current_Sensor cs) = 0;

   private:
    float _current[End_C];
};