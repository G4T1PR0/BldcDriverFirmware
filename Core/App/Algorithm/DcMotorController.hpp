/*
 * BldcController.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <math.h>
#include <DeviceDriver/Interface/baseCurrentSensor.hpp>
#include <DeviceDriver/Interface/baseDCMotorDriver.hpp>
#include <DeviceDriver/Interface/baseEncoder.hpp>
#include <Lib/pid.hpp>

#define LP_FILTER(value, sample, filter_constant) (value -= (filter_constant) * ((value) - (sample)))

class DcMotorController {
   public:
    DcMotorController(baseDCMotorDriver* driver, baseCurrentSensor* currentSensor, baseEncoder* encoder) {
        _driver = driver;
        _currentSensor = currentSensor;
        _encoder = encoder;
    }

    enum Mode {
        Stop,
        VoltageControl,
        CurrentControl,
        VelocityControl,
        Beep,
    };

    void init() {
        _pid_current.setPID(8, 0, 0);

        _pid_velocity.setPID(0.015, 0.000005, 0);
    }

    void update() {
        _encoder->update();
        _currentSensor->update();
        float temp_c = (_currentSensor->getCurrent(baseCurrentSensor::U_C) - _currentSensor->getCurrent(baseCurrentSensor::V_C)) / 2;
        LP_FILTER(_observed_current, temp_c, 0.01);

        LP_FILTER(_observed_velocity, _encoder->getVelocity(), 0.1);

        switch (_mode) {
            case Mode::Stop:
                _driver->setPWM(0);
                break;

            case Mode::VoltageControl:
                _duty = _target_duty;

                if (_driver_enable) {
                    _driver->setPWM(_duty);
                } else {
                    _driver->setPWM(0);
                }

                break;

            case Mode::CurrentControl: {
                _duty = _pid_current.update(_target_current, _observed_current);

                // printf("c: %f %f %f\n", _target_current, _observed_current, _duty);

                if (_target_current > 0) {
                    if (_duty < 0)
                        _duty = 0;
                } else if (_target_current < 0) {
                    if (_duty > 0)
                        _duty = 0;
                } else if (_target_current == 0) {
                    _duty = 0;
                }

                if (_driver_enable) {
                    _driver->setPWM(_duty);
                } else {
                    _driver->setPWM(0);
                }

            } break;

            case Mode::VelocityControl: {
                _target_current = _pid_velocity.update(_target_velocity, _observed_velocity);

                _duty = _pid_current.update(_target_current, _observed_current);

                if (_driver_enable) {
                    _driver->setPWM(_duty);
                } else {
                    _driver->setPWM(0);
                }
            } break;

            case Mode::Beep:
                _beep_cnt_e++;
                _beep_cnt_c++;
                if (_beep_cnt_e > _beep_time) {
                    _driver->setPWM(0);
                    // _beep_cnt = 0;
                    _mode = Mode::Stop;
                } else {
                    if (_beep_cnt_c > (50 / _beep_freq * 1000)) {
                        _beep_cnt_c = 0;
                    } else if (_beep_cnt_c > (50 / _beep_freq * 1000) * 0.5) {
                        _duty = _pid_current.update(_beep_current, _observed_current);
                    } else {
                        _duty = _pid_current.update(-_beep_current, _observed_current);
                    }

                    if (_duty > 0.5) {
                        _duty = 0.5;
                    } else if (_duty < -0.5) {
                        _duty = -0.5;
                    }

                    _driver->setPWM(_duty);
                }
                break;

            default:
                break;
        }
        _driver->update();
    }

    void setEnable(bool enable) {
        _driver_enable = enable;
    }

    void setMode(Mode mode) {
        _mode = mode;
    }

    Mode getMode() {
        return _mode;
    }

    void setTargetDuty(float duty) {
        _target_duty = duty;
    }

    float getTargetDuty() {
        return _target_duty;
    }

    float getApplyVoltage() {
        return _duty;
    }

    float getApplyDuty() {
        return _duty;
    }

    void setTargetCurrent(float current) {
        _target_current = current;
    }

    float getTargetCurrent() {
        return _target_current;
    }

    float getObservedCurrent() {
        return _observed_current;
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
    baseDCMotorDriver* _driver;
    baseCurrentSensor* _currentSensor;
    baseEncoder* _encoder;

    PID<float> _pid_current;

    PID<float> _pid_velocity;

    bool _driver_enable = false;

    Mode _mode;

    float _calib_e_angle = -M_PI * 3 / 2;
    unsigned int calib_cnt = 0;

    float _duty = 0;

    float _target_duty = 0;

    float _target_current = 0;

    float _observed_current = 0;

    float _target_velocity = 0;

    volatile float _observed_velocity = 0;

    unsigned int _beep_cnt_c = 0;
    unsigned int _beep_cnt_e = 0;
    float _beep_freq = 0;
    float _beep_current = 0;
    float _beep_time = 0;
};