/*
 * app_main.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#include "app_main.h"
#include <Algorithm/AngleProcessor.hpp>
#include <Algorithm/BldcController.hpp>
#include <Algorithm/CurrentProcessor.hpp>
#include <Algorithm/ModulationProcessor.hpp>
#include <DeviceDriver/BLDCDriverMCU.hpp>
#include <DeviceDriver/CurrentSensorMCU.hpp>
#include <DeviceDriver/EncoderMCU.hpp>
#include <McuAbstractionLayer/stm32halAbstractionLayer.hpp>

stm32halAbstractionLayer mcu;

CurrentSensorMCU currentSensor(&mcu, MAL::P_ADC::U_Current, MAL::P_ADC::V_Current, MAL::P_ADC::W_Current);
BLDCDriverMCU driver(&mcu, MAL::P_PWM::U_PWM, MAL::P_PWM::V_PWM, MAL::P_PWM::W_PWM);
EncoderMCU encoder(&mcu, MAL::P_Encoder::Main_Encoder);

AngleProcessor angleProcessor(&encoder);
CurrentProcessor currentProcessor(&mcu, &currentSensor);
ModulationProcessor modulationProcessor(&mcu, &driver);
BldcController bldcController(&angleProcessor, &currentProcessor, &modulationProcessor);

unsigned int print_cnt = 0;
unsigned int mode_cnt = 0;
unsigned int timer_1ms_cnt = 0;
float process_time = 0;

void app_interrupt_20us();

void app_init() {
    mcu.init();

    currentSensor.init();
    driver.init();
    encoder.init();

    angleProcessor.init();
    modulationProcessor.init();
    currentProcessor.init();

    bldcController.init();

    mcu.interruptSetCallback(MAL::P_Interrupt::T20us, app_interrupt_20us);

    mcu.waitMs(500);
    printf("\033[H");
    printf("\033[2J");
    printf(R"EOF(
          ____            ____ ____  _         
 _ __ ___|  _ \ ___      / ___/ ___|| |        
| '__/ _ \ |_) / _ \ ____\___ \___ \| |        
| | |  __/  _ < (_) |_____|__) |__) | |___ 
|_|  \___|_| \_\___/     |____/____/|_____|
                                               
 ____  _     _      ____       _                
| __ )| | __| | ___|  _ \ _ __(_)_   _____ _ __ 
|  _ \| |/ _` |/ __| | | | '__| \ \ / / _ \ '__|
| |_) | | (_| | (__| |_| | |  | |\ V /  __/ |   
|____/|_|\__,_|\___|____/|_|  |_| \_/ \___|_|   
                                          
)EOF");

    printf("\x1b[32m[Main Thread]\x1b[39m Initialization Complete\n");
}

void app_main() {
    app_init();

    mcu.gpioSetValue(MAL::P_GPIO::Driver_Power_Switch, true);

    printf("\x1b[32m[Main Thread]\x1b[39m Calibration Start\n");

    bldcController.setMode(BldcController::Mode::Calibration1);

    while (bldcController.getMode() != BldcController::Mode::Stop) {
    }

    printf("\x1b[32m[Main Thread]\x1b[39m Calibration End\n");

    mcu.waitMs(300);
    bldcController.beep(2045, 0.08, 250);
    mcu.waitMs(400);
    bldcController.beep(3500, 0.08, 100);
    mcu.waitMs(150);
    bldcController.beep(3500, 0.08, 100);
    mcu.waitMs(1000);

    // bldcController.setMode(BldcController::Mode::VoltageControl);
    bldcController.setMode(BldcController::Mode::CurrentControl);
    // bldcController.setMode(BldcController::Mode::VelocityControl);

    // bldcController.setTargetCurrent(0, 0);
    // bldcController.setTargetVelocity(0);

    bool dir = false;

    while (1) {
        // if (mode_cnt > 10000) {
        //     mode_cnt = 0;
        // } else if (mode_cnt > 5100) {
        //     // bldcController.setTargetVoltage(-5, 0);
        //     // bldcController.setTargetCurrent(-0.11, 0);
        //     bldcController.setTargetVelocity(60);
        // } else if (mode_cnt > 5000) {
        //     // bldcController.setTargetVoltage(-5, 0);
        //     // bldcController.setTargetCurrent(-0.11, 0);
        //     bldcController.setTargetVelocity(60);
        // } else if (mode_cnt > 100) {
        //     // bldcController.setTargetVoltage(5, 0);
        //     // bldcController.setTargetCurrent(0.11, 0);
        //     bldcController.setTargetVelocity(6.28);
        // } else {
        //     // bldcController.setTargetVoltage(5, 0);
        //     // bldcController.setTargetCurrent(0.11, 0);
        //     bldcController.setTargetVelocity(6.28);
        // }

        if (dir) {
            bldcController.setTargetCurrent(0.4, 0);
            if (bldcController.getObservedVelocity() > 350) {
                dir = false;
            }
        } else {
            bldcController.setTargetCurrent(-0.4, 0);
            if (bldcController.getObservedVelocity() < -350) {
                dir = true;
            }
        }

        // bldcController.setTargetCurrent(0.7, 0);

        if (print_cnt > 100) {
            // printf("e_cnt %8ld ", encoder.getTotalCnt());
            // printf("t_v %5.2f ", bldcController.getTargetVelocity());
            // printf("o_v %5.2f ", bldcController.getObservedVelocity());
            // printf("p_time %4.2fus\n", process_time);

            ////

            // printf("encoder %ld ", encoder.getTotalCnt());
            // printf("e_a %f ", angleProcessor.getElectricalAngle());
            // printf("m_a %f ", angleProcessor.getMechanicalAngle());
            // printf("v1 %f ", encoder.getVelocity());
            printf("rad/s %.2f ", bldcController.getObservedVelocity());
            printf("rpm %.2f ", bldcController.getObservedVelocity() * 60 / (2 * M_PI));

            // float u, v, w;
            // modulationProcessor.getDuty(u, v, w);
            // printf("u: %.2f, v: %.2f, w: %.2f ", u, v, w);

            // CurrentProcessor::PhaseCurrent_s current = currentProcessor.getPhaseCurrent();

            // printf("uc: %.2f, vc: %.2f, wc: %.2f ", current.u, current.v, current.w);

            // printf("a: %f b: %f ", currentProcessor.getAlphaBetaCurrent().alpha, currentProcessor.getAlphaBetaCurrent().beta);

            // float t_qc, t_dc;

            // bldcController.getTargetCurrent(t_qc, t_dc);

            // printf("t_qc: %.2f t_dc: %.2f ", t_qc, t_dc);
            // float vd;
            // float vq;

            // bldcController.getApplyVoltage(vq, vd);

            // printf("vd: %.2f vq: %.2f ", vd, vq);
            printf("od: %.2f oq: %.2f ", currentProcessor.getDQCurrent().d, currentProcessor.getDQCurrent().q);

            printf("p_time %.2f\n", process_time);

            print_cnt = 0;
        }
    }
}

void app_interrupt_20us() {  // 50kHz
    mcu.timerSetCnt(MAL::P_TimerCnt::C1, 0);
    bldcController.update();

    timer_1ms_cnt++;
    if (timer_1ms_cnt >= 50) {  // 1kHz
        encoder.update1kHz();
        print_cnt++;
        mode_cnt++;
        timer_1ms_cnt = 0;
    }

    process_time = float(mcu.timerGetCnt(MAL::P_TimerCnt::C1)) * 1 / 275000000 * 1000000;
}