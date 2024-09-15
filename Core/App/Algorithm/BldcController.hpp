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
        RUN,
    };

    void init() {
    }

    void update() {
        _angleProcessor->update();
        _currentProcessor->update(_angleProcessor->getElectricalAngle());

        switch (_mode) {
            case Mode::UnInitilized:
                break;

            case Mode::Calibration1: {
                _modulationProcessor->setVoltage(5, 0, _calib_e_angle);
                _calib_e_angle += 0.0005;
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
                _modulationProcessor->setVoltage(_voltage_q, _voltage_d, _angleProcessor->getElectricalAngle());
                break;

            case Mode::CurrentControl:
                break;

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

    void setVoltage(float voltage_q, float voltage_d) {
        _voltage_q = voltage_q;
        _voltage_d = voltage_d;
    }

   private:
    CurrentProcessor* _currentProcessor;
    AngleProcessor* _angleProcessor;
    ModulationProcessor* _modulationProcessor;

    Mode _mode;

    float _calib_e_angle = 0;
    unsigned int calib_cnt = 0;

    float _voltage_q = 0;
    float _voltage_d = 0;
};