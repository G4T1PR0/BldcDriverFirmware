/*
 * DCMotorDriverMCU.hpp
 *
 *  Created on: Nov 8, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <DeviceDriver/Interface/baseDCMotorDriver.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class DCMotorDriverMCU : public baseDCMotorDriver {
   public:
    DCMotorDriverMCU(MAL* mcu, MAL::P_PWM hb1, MAL::P_PWM hb2);
    void init(void);
    void update(void);
    void setPWM(float duty);
    void enable(bool);

   private:
    MAL* _mcu;
    MAL::P_PWM _hb1;
    MAL::P_PWM _hb2;

    bool _enabled = false;

    float _hb1_duty = 0;
    float _hb2_duty = 0;

    float _prev_hb1_duty = 0;
    float _prev_hb2_duty = 0;
};