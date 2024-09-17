/*
 * CurrentSensorMCU.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#include <DeviceDriver/CurrentSensorMCU.hpp>

CurrentSensorMCU::CurrentSensorMCU(MAL* mcu, MAL::P_ADC u, MAL::P_ADC v, MAL::P_ADC w) {
    _mcu = mcu;
    _u = u;
    _v = v;
    _w = w;
}

void CurrentSensorMCU::init(void) {
    _currentSensorOffset_Sum[U_C] = 0;
    _currentSensorOffset_Sum[V_C] = 0;
    _currentSensorOffset_Sum[W_C] = 0;

    for (int i = 0; i < _currentSensorOffset_NUM; i++) {
        _currentSensorOffset_Sum[U_C] += _mcu->adcGetValue(_u);
        _currentSensorOffset_Sum[V_C] += _mcu->adcGetValue(_v);
        _currentSensorOffset_Sum[W_C] += _mcu->adcGetValue(_w);
        _mcu->waitMs(1);
    }

    _currentSensorOffset[U_C] = _currentSensorOffset_Sum[U_C] / _currentSensorOffset_NUM;
    _currentSensorOffset[V_C] = _currentSensorOffset_Sum[V_C] / _currentSensorOffset_NUM;
    _currentSensorOffset[W_C] = _currentSensorOffset_Sum[W_C] / _currentSensorOffset_NUM;
}

void CurrentSensorMCU::update(void) {
    _current[U_C] = (_mcu->adcGetValue(_u) - _currentSensorOffset[U_C]) * _raw2voltage16bit / _voltage2current;
    _current[V_C] = (_mcu->adcGetValue(_v) - _currentSensorOffset[V_C]) * _raw2voltage16bit / _voltage2current;
    _current[W_C] = (_mcu->adcGetValue(_w) - _currentSensorOffset[W_C]) * _raw2voltage12bit / _voltage2current;
}

float CurrentSensorMCU::getCurrent(Current_Sensor cs) {
    return _current[cs];
}