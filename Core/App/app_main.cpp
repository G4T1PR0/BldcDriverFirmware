/*
 * app_main.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#include "app_main.h"
#include <Algorithm/AngleProcessor.hpp>
#include <Algorithm/BldcController.hpp>
#include <Algorithm/CommandReceiver.hpp>
#include <Algorithm/CurrentProcessor.hpp>
#include <Algorithm/ModulationProcessor.hpp>
#include <DeviceDriver/BLDCDriverMCU.hpp>
#include <DeviceDriver/CurrentSensorMCU.hpp>
#include <DeviceDriver/DCMotorDriver.hpp>
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

CommandReceiver commandReceiver(&mcu, MAL::P_UART::Controller, &bldcController);

unsigned int feedback_cnt = 0;
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
    angleProcessor.setDirection(true);
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

    mcu.waitMs(500);
    bldcController.beep(2045, 0.1, 350);
    mcu.waitMs(500);

    printf("\x1b[32m[Main Thread]\x1b[39m Calibration Start\n");

    bldcController.setMode(BldcController::Mode::Calibration1);

    while (bldcController.getMode() != BldcController::Mode::Stop) {
    }

    printf("\x1b[32m[Main Thread]\x1b[39m Calibration End\n");

    mcu.waitMs(300);
    bldcController.beep(2045, 0.1, 250);
    mcu.waitMs(400);
    bldcController.beep(3500, 0.1, 100);
    mcu.waitMs(150);
    bldcController.beep(3500, 0.1, 100);
    mcu.waitMs(1000);

    bldcController.setMode(BldcController::Mode::Stop);

    // bldcController.setEnable(true);
    // bldcController.setMode(BldcController::Mode::CurrentControl);
    // bldcController.setTargetCurrent(0.8, 0);

    while (1) {
        commandReceiver.update();

        if (feedback_cnt > 5) {
            commandReceiver.send();
        }

        if (print_cnt > 100) {
            // printf("\x1b[32m[Main Thread]\x1b[39m p_time: %f\n", process_time);
            // printf("\x1b[32m[Main Thread]\x1b[39m Mode: %d\n", bldcController.getMode());
            // printf("\x1b[32m[Main Thread]\x1b[39m C_A %f C_B %f C_C %f\n", currentSensor.getCurrent(CurrentSensorMCU::U_C), currentSensor.getCurrent(CurrentSensorMCU::V_C), currentSensor.getCurrent(CurrentSensorMCU::W_C));
            // printf("\x1b[32m[Main Thread]\x1b[39m enc: %f\n", encoder.getAngle());

            // printf("e_cnt %8ld ", encoder.getTotalCnt());
            // printf("t_v %5.2f ", bldcController.getTargetVelocity());
            // printf("o_v %5.2f ", bldcController.getObservedVelocity());
            // printf("p_time %4.2fus\n", process_time);

            ////

            // printf("encoder %ld ", encoder.getTotalCnt());
            // printf("e_a %f ", angleProcessor.getElectricalAngle());
            // printf("m_a %f ", angleProcessor.getMechanicalAngle());
            // printf("v1 %f ", encoder.getVelocity());
            // printf("rad/s %.2f ", bldcController.getObservedVelocity());
            // printf("rpm %.2f ", bldcController.getObservedVelocity() * 60 / (2 * M_PI));
            // printf("cnt20us %ld ", encoder.getCnt());

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
            // printf("od: %.2f oq: %.2f ", currentProcessor.getDQCurrent().d, currentProcessor.getDQCurrent().q);

            // printf("p_time %.2f\n", process_time);
            // print_cnt = 0;
        }
    }
}

void app_interrupt_20us() {  // 50kHz
    mcu.timerSetCnt(MAL::P_TimerCnt::C1, 0);
    bldcController.update();

    timer_1ms_cnt++;
    if (timer_1ms_cnt >= 50) {  // 1kHz
        encoder.update1kHz();
        commandReceiver.cnt++;
        feedback_cnt++;
        // print_cnt++;
        mode_cnt++;
        timer_1ms_cnt = 0;
    }

    process_time = float(mcu.timerGetCnt(MAL::P_TimerCnt::C1));
}