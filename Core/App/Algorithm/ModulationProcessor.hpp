/*
 * ModulationProcessor.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <math.h>
#include <DeviceDriver/Interface/baseBLDCDriver.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class ModulationProcessor {
   public:
    ModulationProcessor(baseMcuAbstractionLayer* mcu, baseBLDCDriver* driver) {
        _mcu = mcu;
        _driver = driver;
    }

    void init() {
    }

    void update() {
        float cos;
        float sin;

        //_mcu->cordicSinCos(_electric_angle, &sin, &cos);

        float angle_el = _normalizeAngle(_electric_angle + M_PI / 2);

        sin = sinf(angle_el);
        cos = cosf(angle_el);

        // sin = sinf(_electric_angle);
        // cos = cosf(_electric_angle);

        float alpha = cos * _voltage_d - sin * _voltage_q;
        float beta = sin * _voltage_d + cos * _voltage_q;

        float Va = alpha;
        float Vb = -0.5f * alpha + _sqrt3_2 * beta;
        float Vc = -0.5f * alpha - _sqrt3_2 * beta;

        float center = _voltage_limit / 2;

        float Vmin = _min(Va, _min(Vb, Vc));
        float Vmax = _max(Va, _max(Vb, Vc));

        center -= (Vmax + Vmin) / 2;

        float VVmin = _min(Va, _min(Vb, Vc));
        Va -= VVmin;
        Vb -= VVmin;
        Vc -= VVmin;

        float u_pwm = Va / _voltage_limit;
        float v_pwm = Vb / _voltage_limit;
        float w_pwm = Vc / _voltage_limit;

        _driver->enable(true);
        _driver->setPWM(u_pwm, v_pwm, w_pwm);
        _driver->update();

        // printf("U_PWM: %f, V_PWM: %f, W_PWM: %f\n", u_pwm, v_pwm, w_pwm);
    }

    void setVoltage(float voltage_q, float voltage_d, float electric_angle) {
        _voltage_q = voltage_q;
        _voltage_d = voltage_d;
        _electric_angle = electric_angle;
    }

    void setVoltageLimit(float voltage_limit) {
        _voltage_limit = voltage_limit;
    }

   private:
    baseMcuAbstractionLayer* _mcu;
    baseBLDCDriver* _driver;

    float _voltage_q;
    float _voltage_d;
    float _electric_angle;

    float _voltage_limit = 12;

    const float _sqrt3_2 = sqrt(3) / 2;

    float _min(float a, float b) {
        return a < b ? a : b;
    }

    float _max(float a, float b) {
        return a > b ? a : b;
    }

    float _normalizeAngle(float angle) {
        float a = fmod(angle, 2 * M_PI);

        if (a < 0) {
            a += 2 * M_PI;
        }

        return a;
    }
};