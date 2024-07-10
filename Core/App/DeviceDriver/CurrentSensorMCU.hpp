/*
 * CurrentSensorMCU.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <DeviceDriver/Interface/baseCurrentSensor.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class CurrentSensorMCU : public baseCurrentSensor {
   public:
    CurrentSensorMCU(MAL* mcu, MAL::P_ADC u, MAL::P_ADC v, MAL::P_ADC w);
    void init(void);
    void update(void);
    float getCurrent(Current_Sensor cs);

   private:
    MAL* _mcu;
    MAL::P_ADC _u;
    MAL::P_ADC _v;
    MAL::P_ADC _w;

    float _current[End_C];

    const float _raw2voltage = 3.3f / (1 << 12);
    const float _voltage2current = 0.033f;

    const int _currentSensorOffset_NUM = 1000;
    float _currentSensorOffset_Sum[End_C];
    float _currentSensorOffset[End_C];
};