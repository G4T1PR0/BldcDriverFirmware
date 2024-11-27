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
        _pid_current_q.setPID(5, 0, 0);
        _pid_current_d.setPID(5, 0, 0);

        _pid_velocity.setPID(0.015, 0.000005, 0);
    }

    void update() {
        _angleProcessor->update();
        _currentProcessor->update(_angleProcessor->getElectricalAngle());

        LP_FILTER(_observed_current_d, _currentProcessor->getDQCurrent().d, 0.2);
        LP_FILTER(_observed_current_q, _currentProcessor->getDQCurrent().q, 0.2);

        LP_FILTER(_observed_velocity, _angleProcessor->getVelocity(), 0.1);

        switch (_mode) {
            case Mode::UnInitilized:
                break;

            case Mode::Calibration1: {
                _modulationProcessor->setVoltage(0.5, 0, _calib_e_angle);
                _calib_e_angle += 0.00005;
                if (_calib_e_angle >= M_PI * 3 / 2) {
                    _modulationProcessor->setVoltage(5, 0, M_PI * 3 / 2);
                    _mode = Mode::Calibration2;
                }
            } break;

            case Mode::Calibration2:
                _modulationProcessor->setVoltage(0.5, 0, M_PI * 3 / 2);

                calib_cnt++;
                if (calib_cnt > 1500 * 50) {
                    _mode = Mode::Calibration3;
                }
                break;

            case Calibration3:
                _modulationProcessor->setVoltage(0.5, 0, M_PI * 3 / 2);
                _angleProcessor->setZero();
                _mode = Mode::Stop;
                break;

            case Mode::Stop:
                _modulationProcessor->setVoltage(0, 0, _angleProcessor->getElectricalAngle());
                break;

            case Mode::VoltageControl:
                _voltage_q = _target_voltage_q;
                _voltage_d = _target_voltage_d;

                if (_driver_enable) {
                    _modulationProcessor->setVoltage(_voltage_q, _voltage_d, _angleProcessor->getElectricalAngle());
                } else {
                    _modulationProcessor->setVoltage(0, 0, _angleProcessor->getElectricalAngle());
                }

                break;

            case Mode::CurrentControl: {
                _voltage_q = _pid_current_q.update(_target_current_q, _observed_current_q);
                _voltage_d = _pid_current_d.update(_target_current_d, _observed_current_d);

                if (_driver_enable) {
                    _modulationProcessor->setVoltage(_voltage_q, _voltage_d, _angleProcessor->getElectricalAngle());
                } else {
                    _modulationProcessor->setVoltage(0, 0, _angleProcessor->getElectricalAngle());
                }

            } break;

            case Mode::VelocityControl: {
                _target_current_q = _pid_velocity.update(_target_velocity, _observed_velocity);
                _target_current_d = 0;

                _voltage_q = _pid_current_q.update(_target_current_q, _observed_current_q);
                _voltage_d = _pid_current_d.update(_target_current_d, _observed_current_d);

                if (_driver_enable) {
                    _modulationProcessor->setVoltage(_voltage_q, _voltage_d, _angleProcessor->getElectricalAngle());
                } else {
                    _modulationProcessor->setVoltage(0, 0, _angleProcessor->getElectricalAngle());
                }
            } break;

            case Mode::Beep:
                _beep_cnt_e++;
                _beep_cnt_c++;
                if (_beep_cnt_e > _beep_time) {
                    _modulationProcessor->setVoltage(0, 0, _angleProcessor->getElectricalAngle());
                    // _beep_cnt = 0;
                    _mode = Mode::Stop;
                } else {
                    if (_beep_cnt_c > (50 / _beep_freq * 1000)) {
                        _beep_cnt_c = 0;
                    } else if (_beep_cnt_c > (50 / _beep_freq * 1000) * 0.5) {
                        _voltage_q = _pid_current_q.update(_beep_current, _observed_current_q);
                    } else {
                        _voltage_q = _pid_current_q.update(0, _observed_current_q);
                    }

                    _voltage_d = _pid_current_d.update(0, _observed_current_d);

                    if (_voltage_q > 4) {
                        _voltage_q = 4;
                    } else if (_voltage_q < -4) {
                        _voltage_q = -4;
                    }

                    _modulationProcessor->setVoltage(_voltage_q, _voltage_d, _angleProcessor->getElectricalAngle());
                }
                break;

            default:
                break;
        }

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

    float _voltage_q = 0;
    float _voltage_d = 0;

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
};