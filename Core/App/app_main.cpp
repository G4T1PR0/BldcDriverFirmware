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

    bldcController.setMode(BldcController::Mode::CurrentControl);
    bldcController.setTargetCurrent(0.3, 0);

    while (1) {
        if (mode_cnt > 2000) {
            mode_cnt = 0;
        } else if (mode_cnt > 1000) {
            // bldcController.setTargetVoltage(11, 0);
            bldcController.setTargetCurrent(0.1, 0);
        } else {
            // bldcController.setTargetVoltage(0, 0);
            bldcController.setTargetCurrent(-0.1, 0);
        }

        if (print_cnt > 100) {
            printf("encoder %ld ", encoder.getTotalCnt());
            printf("e_a %f ", angleProcessor.getElectricalAngle());

            float u, v, w;
            modulationProcessor.getDuty(u, v, w);
            printf("u: %f, v: %f, w: %f ", u, v, w);

            // CurrentProcessor::PhaseCurrent_s current = currentProcessor.getPhaseCurrent();

            // printf("uc: %f, vc: %f, wc: %f ", current.u, current.v, current.w);

            // currentSensor.getCurrent(baseCurrentSensor::U_C);

            // printf("uc: %f, vc: %f, wc: %f ", currentSensor.getCurrent(baseCurrentSensor::U_C), currentSensor.getCurrent(baseCurrentSensor::V_C), currentSensor.getCurrent(baseCurrentSensor::W_C));

            // printf("a: %f b: %f ", currentProcessor.getAlphaBetaCurrent().alpha, currentProcessor.getAlphaBetaCurrent().beta);

            float vd;
            float vq;

            bldcController.getApplyVoltage(vq, vd);

            printf("vd: %f vq: %f ", vd, vq);
            printf("od: %f oq: %f ", currentProcessor.getDQCurrent().d, currentProcessor.getDQCurrent().q);

            printf("p_time %f\n", process_time);
            print_cnt = 0;
        }
    }
}

void app_interrupt_20us() {  // 50kHz
    mcu.timerSetCnt(MAL::P_TimerCnt::C1, 0);
    bldcController.update();

    if (timer_1ms_cnt > 50) {
        print_cnt++;
        mode_cnt++;
        timer_1ms_cnt = 0;
    }
    timer_1ms_cnt++;

    process_time = float(mcu.timerGetCnt(MAL::P_TimerCnt::C1)) * 1 / 275000000 * 1000000;
}