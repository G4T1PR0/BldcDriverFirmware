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
        Stop,
        VoltageControl,
        CurrentControl,
        RUN,
    };

    void init() {
    }

    void update() {
        _angleProcessor->update();
        // _currentProcessor->update(_angleProcessor->getElectricalAngle());

        switch (_mode) {
            case Mode::UnInitilized:
                break;

            case Mode::Calibration1: {
                _modulationProcessor->setVoltage(5, 0, _calib_e_angle);
                _calib_e_angle += 0.0005;
                if (_calib_e_angle > 6.28) {
                    _mode = Mode::Calibration2;
                }
            } break;

            case Mode::Calibration2:
                _modulationProcessor->setVoltage(5, 0, 0);
                _angleProcessor->setZero();
                _mode = Mode::Stop;
                break;

            case Mode::Stop:
                _modulationProcessor->setVoltage(0, 0, 0);
                break;

            case Mode::VoltageControl:
                _modulationProcessor->setVoltage(2, 0, _angleProcessor->getElectricalAngle());
                break;

            default:
                break;
        }

        _modulationProcessor->update();
    }

    void setMode(Mode mode) {
        _mode = mode;
    }

    Mode getMode() {
        return _mode;
    }

   private:
    CurrentProcessor* _currentProcessor;
    AngleProcessor* _angleProcessor;
    ModulationProcessor* _modulationProcessor;

    Mode _mode;

    float _calib_e_angle = 0;
};