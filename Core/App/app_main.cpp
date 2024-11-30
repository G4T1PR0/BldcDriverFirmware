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

    // while (1) {
    //     printf("test1\n");
    //     mcu.waitMs(1000);
    // }

    // mcu.waitMs(5000);

    mcu.gpioSetValue(MAL::P_GPIO::Driver_Power_Switch, false);

    // while (1) {
    //     printf("test2\n");
    //     mcu.waitMs(1000);
    // }

    mcu.waitMs(500);
    bldcController.beep(2045, 0.5, 350);
    mcu.waitMs(500);

    printf("\x1b[32m[Main Thread]\x1b[39m Calibration Start\n");

    bldcController.setMode(BldcController::Mode::Calibration1);

    while (bldcController.getMode() != BldcController::Mode::Stop) {
    }

    printf("\x1b[32m[Main Thread]\x1b[39m Calibration End\n");

    mcu.waitMs(300);
    bldcController.beep(2045, 0.5, 250);
    mcu.waitMs(400);
    bldcController.beep(3500, 0.5, 100);
    mcu.waitMs(150);
    bldcController.beep(3500, 0.5, 100);
    mcu.waitMs(1000);

    bldcController.setMode(BldcController::Mode::Stop);

    // bldcController.setEnable(true);
    // bldcController.setMode(BldcController::Mode::CurrentControl);
    // bldcController.setTargetCurrent(0.5, 0);

    // bldcController.setEnable(true);
    // bldcController.setMode(BldcController::Mode::VoltageControl);
    // bldcController.setTargetVoltage(1, 0);

    int mode = 0;

    while (1) {
        // switch (mode) {
        //     case 0:
        //         bldcController.setEnable(true);
        //         bldcController.setTargetCurrent(1.2, 0);
        //         if (angleProcessor.getMechanicalAngle() > 1000) {
        //             mode = 1;
        //         }
        //         break;

        //     case 1:
        //         bldcController.setEnable(false);
        //         if (bldcController.getObservedVelocity() < 10) {
        //             mode = 2;
        //         }
        //         break;

        //     case 2:
        //         bldcController.setEnable(true);
        //         bldcController.setTargetCurrent(-1.2, 0);
        //         if (angleProcessor.getMechanicalAngle() < -1000) {
        //             mode = 3;
        //         }

        //         break;

        //     case 3:
        //         bldcController.setEnable(false);
        //         if (bldcController.getObservedVelocity() > -10) {
        //             mode = 0;
        //         }
        //         break;

        //     default:
        //         break;
        // }

        // commandReceiver.update();

        // if (feedback_cnt > 5) {
        //     commandReceiver.send();
        // }

        if (print_cnt > 50) {
            // printf("\x1b[32m[Main Thread]\x1b[39m p_time: %f\n", process_time);
            // printf("\x1b[32m[Main Thread]\x1b[39m Mode: %d\n", bldcController.getMode());
            // printf("\x1b[32m[Main Thread]\x1b[39m C_A %f C_B %f C_C %f\n", currentSensor.getCurrent(CurrentSensorMCU::U_C), currentSensor.getCurrent(CurrentSensorMCU::V_C), currentSensor.getCurrent(CurrentSensorMCU::W_C));
            // printf("\x1b[32m[Main Thread]\x1b[39m enc: %f\n", encoder.getAngle());

            // printf(">e_cnt: %ld ", encoder.getTotalCnt());
            // printf(">t_v: %5.2f ", bldcController.getTargetVelocity());
            // printf("o_v: %f ", bldcController.getObservedVelocity());
            // printf("p_time %4.2fus\n", process_time);

            ////

            // printf("b_vr: %f ", mcu.adcGetValue(MAL::P_ADC::Bus_Voltage) * 3.3f / (1 << 16));

            printf("mode: %d ", mode);

            printf("b_v: %f ", mcu.adcGetValue(MAL::P_ADC::Bus_Voltage) * 3.3f / (1 << 16) * 23.25 / 3.3);

            printf("encoder %ld ", encoder.getTotalCnt());
            // printf(">e_a: %f ", angleProcessor.getElectricalAngle());
            printf(">m_a: %f ", angleProcessor.getMechanicalAngle());
            // printf("v1 %f ", encoder.getVelocity());
            printf("rad/s %.2f ", bldcController.getObservedVelocity());
            printf("rpm %.2f ", bldcController.getObservedVelocity() * 60 / (2 * M_PI));
            // printf("cnt20us %ld ", encoder.getCnt());

            // float u, v, w;
            // modulationProcessor.getDuty(u, v, w);
            // printf(">u_duty: %.2f >v_duty: %.2f >w_duty: %.2f ", u, v, w);

            // CurrentProcessor::PhaseCurrent_s current = currentProcessor.getPhaseCurrent();

            // printf(">u_current: %.2f v_current: %.2f >w_current: %.2f ", current.u, current.v, current.w);

            // printf("a: %f b: %f ", currentProcessor.getAlphaBetaCurrent().alpha, currentProcessor.getAlphaBetaCurrent().beta);

            // float t_qc, t_dc;

            // bldcController.getTargetCurrent(t_qc, t_dc);

            // printf("t_qc: %.2f t_dc: %.2f ", t_qc, t_dc);

            float vd;
            float vq;

            bldcController.getApplyVoltage(vq, vd);

            printf("d_voltage: %.2f q_voltage: %.2f ", vd, vq);
            printf("d_o_current: %.2f q_o_current: %.2f ", currentProcessor.getDQCurrent().d, currentProcessor.getDQCurrent().q);

            printf("p_time: %.2f\n", process_time);

            // Teleplot

            // printf(">e_cnt: %ld\n", encoder.getTotalCnt());
            // printf(">e_a: %f\n", angleProcessor.getElectricalAngle());
            // printf(">m_a: %f\n", angleProcessor.getMechanicalAngle());
            // printf(">rpm: %f\n", bldcController.getObservedVelocity() * 60 / (2 * M_PI));

            // CurrentProcessor::PhaseCurrent_s current = currentProcessor.getPhaseCurrent();

            // printf(">u_c: %f\n", current.u);
            // printf(">v_c: %f\n", current.v);
            // printf(">w_c: %f\n", current.w);

            // printf(">a_c: %f\n", currentProcessor.getAlphaBetaCurrent().alpha);
            // printf(">b_c: %f\n", currentProcessor.getAlphaBetaCurrent().beta);

            // printf(">d_o_current: %f\n", currentProcessor.getDQCurrent().d);
            // printf(">q_o_current: %f\n", currentProcessor.getDQCurrent().q);

            // float d_u, d_v, d_w;
            // modulationProcessor.getDuty(d_u, d_v, d_w);

            // printf(">d_u: %f\n", d_u);
            // printf(">d_v: %f\n", d_v);
            // printf(">d_w: %f\n", d_w);

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
        commandReceiver.cnt++;
        feedback_cnt++;
        print_cnt++;
        mode_cnt++;
        timer_1ms_cnt = 0;
    }

    process_time = float(mcu.timerGetCnt(MAL::P_TimerCnt::C1));
}