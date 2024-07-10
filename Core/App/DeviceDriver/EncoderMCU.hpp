/*
 * EncoderMCU.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <DeviceDriver/Interface/baseEncoder.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class EncoderMCU : public baseEncoder {
   public:
    EncoderMCU(MAL* mcu, MAL::P_Encoder encoder);
    void init();
    void update();
    int32_t getCnt();

   private:
    MAL* _mcu;
    MAL::P_Encoder _encoder;

    const uint32_t _offset = 32767;

    int32_t _cnt;
};
