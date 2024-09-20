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
    };

    void init() {
        _pid_current_q.setPID(48, 0, 0);
        _pid_current_d.setPID(10, 0, 0);

        _pid_velocity.setPID(0.001, 0.00001, 0);
    }

    void update() {
        _angleProcessor->update();
        _currentProcessor->update(_angleProcessor->getElectricalAngle());

        LP_FILTER(_observed_current_d, _currentProcessor->getDQCurrent().d, 0.05);
        LP_FILTER(_observed_current_q, _currentProcessor->getDQCurrent().q, 0.05);

        LP_FILTER(_observed_velocity, _angleProcessor->getVelocity(), 0.1);

        switch (_mode) {
            case Mode::UnInitilized:
                break;

            case Mode::Calibration1: {
                _modulationProcessor->setVoltage(5, 0, _calib_e_angle);
                _calib_e_angle += 0.00025;
                if (_calib_e_angle >= M_PI * 3 / 2) {
                    _modulationProcessor->setVoltage(5, 0, M_PI * 3 / 2);
                    _mode = Mode::Calibration2;
                }
            } break;

            case Mode::Calibration2:
                _modulationProcessor->setVoltage(5, 0, M_PI * 3 / 2);

                calib_cnt++;
                if (calib_cnt > 700 * 20) {
                    _mode = Mode::Calibration3;
                }
                break;

            case Calibration3:
                _modulationProcessor->setVoltage(5, 0, M_PI * 3 / 2);
                _angleProcessor->setZero();
                _mode = Mode::Stop;
                break;

            case Mode::Stop:
                _modulationProcessor->setVoltage(0, 0, 0);
                break;

            case Mode::VoltageControl:
                _voltage_q = _target_voltage_q;
                _voltage_d = _target_voltage_d;

                _modulationProcessor->setVoltage(_voltage_q, _voltage_d, _angleProcessor->getElectricalAngle());
                break;

            case Mode::CurrentControl: {
                _voltage_q = _pid_current_q.update(_target_current_q, _observed_current_q);
                _voltage_d = _pid_current_d.update(_target_current_d, _observed_current_d);

                _modulationProcessor->setVoltage(_voltage_q, _voltage_d, _angleProcessor->getElectricalAngle());
            } break;

            case Mode::VelocityControl: {
                _target_current_q = _pid_velocity.update(_target_velocity, _observed_velocity);
                _target_current_d = 0;

                _voltage_q = _pid_current_q.update(_target_current_q, _observed_current_q);
                _voltage_d = _pid_current_d.update(_target_current_d, _observed_current_d);

                _modulationProcessor->setVoltage(_voltage_q, _voltage_d, _angleProcessor->getElectricalAngle());
            } break;

            default:
                break;
        }

        _modulationProcessor->update();
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

    void setTargetVelocity(float velocity) {
        _target_velocity = velocity;
    }

    float getTargetVelocity() {
        return _target_velocity;
    }

    float getObservedVelocity() {
        return _observed_velocity;
    }

   private:
    CurrentProcessor* _currentProcessor;
    AngleProcessor* _angleProcessor;
    ModulationProcessor* _modulationProcessor;

    PID<float> _pid_current_q;
    PID<float> _pid_current_d;

    PID<float> _pid_velocity;

    Mode _mode;

    float _calib_e_angle = 0;
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
};