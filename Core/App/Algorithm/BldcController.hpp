/*
 * BldcController.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <Algorithm/AngleProcessor.hpp>
#include <Algorithm/CurrentProcessor.hpp>
#include <Algorithm/ModulationProcessor.hpp>
#include <Lib/pid.hpp>

#define LP_FILTER(value, sample, filter_constant) (value -= (filter_constant) * ((value) - (sample)))
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

class BldcController {
   public:
    BldcController(AngleProcessor* angleProcessor, CurrentProcessor* currentProcessor, ModulationProcessor* modulationProcessor) {
        _angleProcessor = angleProcessor;
        _currentProcessor = currentProcessor;
        _modulationProcessor = modulationProcessor;
    }

    enum Mode {
        UnInitilized,
        Calibration1,
        Calibration2,
        Calibration3,
        Stop,
        VoltageControl,
        CurrentControl,
        VelocityControl,
        Beep,
    };

    void init() {
        _pid_current_q.setPID(0.1, 0, 0);
        _pid_current_d.setPID(0.1, 0, 0);

        _pid_velocity.setPID(0.015, 0.000005, 0);

        _modulationProcessor->setInjectionVoltage(_hfi_voltage);
    }

    void update() {
        _injection_polarity = !_injection_polarity;
        _modulationProcessor->setInjectionPolarity(_injection_polarity);
        _currentProcessor->setInjectionPolarity(_injection_polarity);

        _currentProcessor->update(_electrical_angle);

        LP_FILTER(_observed_current_d, _currentProcessor->getDQCurrent().d, 0.1);
        LP_FILTER(_observed_current_q, _currentProcessor->getDQCurrent().q, 0.1);

        LP_FILTER(_observed_velocity, _angleProcessor->getVelocity(), 0.1);

        if (_last_hfi_voltage != _hfi_voltage || _last_Ts != _Ts || _last_Ld != _Ld || _last_Lq != _Lq) {
            _predivAngleest = 1.0f / (_hfi_voltage * _Ts * (1.0f / _Lq - 1.0f / _Ld));
            _last_hfi_voltage = _hfi_voltage;
            _last_Ts = _Ts;
            _last_Ld = _Ld;
            _last_Lq = _Lq;
            _Ts_div = 1 / _Ts;
        }

        _hfi_curangleest = 0.5f * _currentProcessor->getDQCurrentDiff().q * _predivAngleest;
        _hfi_error = -_hfi_curangleest;
        _hfi_int += _hfi_error * _Ts * _hfi_gain2;
        _hfi_int = _constrain(_hfi_int, -_Ts * 120.0f, _Ts * 120.0f);
        _hfi_out += _hfi_gain1 * _Ts * _hfi_error + _hfi_int;

        _voltage_q_bf = _pid_current_q.update(_target_current_q, _currentProcessor->getDQCurrent().q);
        _voltage_d_bf = _pid_current_d.update(_target_current_d, _currentProcessor->getDQCurrent().d);

        _voltage_q = LP_FILTER(_voltage_q, _voltage_q_bf, 0.8);
        _voltage_d = LP_FILTER(_voltage_d, _voltage_d_bf, 0.8);

        _modulationProcessor->setVoltage(_voltage_q, _voltage_d, _electrical_angle);

        while (_hfi_out < 0) {
            _hfi_out += 2 * M_PI;
        }
        while (_hfi_out >= 2 * M_PI) {
            _hfi_out -= 2 * M_PI;
        }
        _hfi_int = _hfinormalizeAngle(_hfi_int);

        float d_angle = _hfi_out - _electrical_angle;
        if (abs(d_angle) > (0.8f * 2 * M_PI))
            _hfi_full_turns += (d_angle > 0.0f) ? -1.0f : 1.0f;
        _electrical_angle = _hfi_out;

        _modulationProcessor->update();
    }

    void setEnable(bool enable) {
        _driver_enable = enable;
    }

    void setMode(Mode mode) {
        if (mode != Mode::Calibration2 || mode != Mode::Calibration3) {
            _mode = mode;
        } else {
            _mode = Mode::UnInitilized;
        }
    }

    Mode getMode() {
        return _mode;
    }

    void setTargetVoltage(float voltage_q, float voltage_d) {
        _target_voltage_q = voltage_q;
        _target_voltage_d = voltage_d;
    }

    void getTargetVoltage(float& voltage_q, float& voltage_d) {
        voltage_q = _target_voltage_q;
        voltage_d = _target_voltage_d;
    }

    void getApplyVoltage(float& voltage_q, float& voltage_d) {
        voltage_q = _voltage_q;
        voltage_d = _voltage_d;
    }

    float getApplyQVoltage() {
        return _voltage_q;
    }

    float getApplyDVoltage() {
        return _voltage_d;
    }

    void setTargetCurrent(float current_q, float current_d) {
        _target_current_q = current_q;
        _target_current_d = current_d;
    }

    void getTargetCurrent(float& current_q, float& current_d) {
        current_q = _target_current_q;
        current_d = _target_current_d;
    }

    void getObservedCurrentDQ(float& current_q, float& current_d) {
        current_q = _observed_current_q;
        current_d = _observed_current_d;
    }

    float getObservedCurrentD() {
        return _observed_current_d;
    }

    float getObservedCurrentQ() {
        return _observed_current_q;
    }

    void setTargetVelocity(float velocity) {
        _target_velocity = velocity;
    }

    float getTargetVelocity() {
        return _target_velocity;
    }

    float getObservedVelocity() {
        return _observed_velocity;
    }

    void beep(float freq, float current, float time) {
        _beep_freq = freq;
        _beep_current = current;
        _beep_time = time * 50;
        _beep_cnt_c = 0;
        _beep_cnt_e = 0;
        _mode = Mode::Beep;
    }

    float getElectricalAngle() {
        return _electrical_angle;
    }

   private:
    CurrentProcessor* _currentProcessor;
    AngleProcessor* _angleProcessor;
    ModulationProcessor* _modulationProcessor;

    PID<float> _pid_current_q;
    PID<float> _pid_current_d;

    PID<float> _pid_velocity;

    bool _driver_enable = false;

    Mode _mode;

    float _calib_e_angle = -M_PI * 3 / 2;
    unsigned int calib_cnt = 0;

    float _electrical_angle = 0;

    float _Ld = 0.00032;
    float _Lq = 0.00048;

    // float _Ld = 4.345e-6f;
    // float _Lq = 5.475e-6f;

    // float _hfi_gain1 = 100.0f * 2 * M_PI;
    // float _hfi_gain2 = 1.0f * 2 * M_PI;

    float _hfi_gain1 = 750.0f * 2 * M_PI;
    float _hfi_gain2 = 5.0f * 2 * M_PI;

    float _last_Ld = _Ld;
    float _last_Lq = _Lq;

    bool _injection_polarity = false;
    float _Ts = 1.0f / (2.0f * 50000.0f);
    float _last_Ts = _Ts;
    float _Ts_div = 1 / _Ts;
    float _hfi_voltage = 4;
    float _last_hfi_voltage = 0;

    float _predivAngleest = 1.0f / (_hfi_voltage * _Ts * (1.0f / _Lq - 1.0f / _Ld));

    float _hfi_curangleest = 0;
    float _hfi_error = 0;
    float _hfi_int = 0;
    float _hfi_out = 0;
    float _hfi_full_turns = 0;

    float _voltage_q = 0;
    float _voltage_d = 0;

    float _voltage_q_bf = 0;
    float _voltage_d_bf = 0;

    float _target_voltage_q = 0;
    float _target_voltage_d = 0;

    float _target_current_q = 0;
    float _target_current_d = 0;

    float _observed_current_q = 0;
    float _observed_current_d = 0;

    float _target_velocity = 0;

    volatile float _observed_velocity = 0;

    unsigned int _beep_cnt_c = 0;
    unsigned int _beep_cnt_e = 0;
    float _beep_freq = 0;
    float _beep_current = 0;
    float _beep_time = 0;

    float _hfinormalizeAngle(float angle) {
        while (angle < 0) {
            angle += 2 * M_PI;
        }
        while (angle >= 2 * M_PI) {
            angle -= 2 * M_PI;
        }
        return angle;
    }
};