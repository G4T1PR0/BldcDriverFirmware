/*
 * EncoderMCU.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <math.h>
#include <DeviceDriver/Interface/baseEncoder.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class EncoderMCU : public baseEncoder {
   public:
    EncoderMCU(MAL* mcu, MAL::P_Encoder encoder);
    void init();
    void update();
    int32_t getCnt();
    int32_t getTotalCnt();
    float getAngle();
    void setCpr(uint16_t cpr);
    void setZero();

   private:
    MAL* _mcu;
    MAL::P_Encoder _encoder;

    const uint32_t _offset = 32767;

    uint16_t _cpr;
    int32_t _cnt;
    int32_t _total_cnt;
};
