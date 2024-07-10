/*
 * BLDCDriverMCU.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <DeviceDriver/Interface/baseBLDCDriver.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class BLDCDriverMCU : public baseBLDCDriver {
   public:
    BLDCDriverMCU(MAL* mcu, MAL::P_PWM u, MAL::P_PWM v, MAL::P_PWM w);
    void init(void);
    void update(void);
    void setPWM(float u, float v, float w);
    void enable(bool);

   private:
    MAL* _mcu;
    MAL::P_PWM _u;
    MAL::P_PWM _v;
    MAL::P_PWM _w;

    bool _enabled = false;

    float _u_duty = 0;
    float _v_duty = 0;
    float _w_duty = 0;

    float _prev_u_duty = 0;
    float _prev_v_duty = 0;
    float _prev_w_duty = 0;
};