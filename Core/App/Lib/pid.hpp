/*
 * pid.hpp
 * Created on: 2023 4/14
 * Author: G4T1PR0
 */

#pragma once

template <typename T>
class PID {
   public:
    PID() {
        _prev_time = 10;
        _kp = 0;
        _ki = 0;
        _kd = 0;
        _error = 0;
        _prev_error = 0;
        _integral = 0;
        _prev_integral = 0;
        _derivative = 0;
        _output = 0;
    };
    void setPID(T p, T i, T d) {
        _kp = p;
        _ki = i;
        _kd = d;
    }
    void setP(T p) {
        _kp = p;
    }
    void setI(T i) {
        _ki = i;
    }
    void setD(T d) {
        _kd = d;
    }
    void setProcessTime(T time) {
        _prev_time = time;
    }
    T getP() {
        return _kp;
    }
    T getI() {
        return _ki;
    }
    T getD() {
        return _kd;
    }
    T getError() {
        return _error;
    }
    T getIntegral() {
        return _integral;
    }
    T getDerivative() {
        return _derivative;
    }
    T getOutput() {
        return _output;
    }

    T update(T target, T feedback) {
        _error = target - feedback;
        _integral += _error;
        _derivative = (_error - _prev_error) / (_prev_time) * 0.001;
        _prev_error = _error;
        _output = _kp * _error + _ki * _integral + _kd * _derivative;
        return _output;
    }

   private:
    T _prev_time;
    T _kp, _ki, _kd;
    T _error, _prev_error, _integral, _prev_integral, _derivative, _output;
};
