/*
 * app_main.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#include "app_main.h"

#include <Algorithm/CommandReceiver.hpp>
#include <Algorithm/DcMotorController.hpp>

#include <DeviceDriver/CurrentSensorMCU.hpp>
#include <DeviceDriver/DCMotorDriver.hpp>
#include <DeviceDriver/EncoderMCU.hpp>
#include <McuAbstractionLayer/stm32halAbstractionLayer.hpp>

stm32halAbstractionLayer mcu;

CurrentSensorMCU currentSensor(&mcu, MAL::P_ADC::U_Current, MAL::P_ADC::V_Current, MAL::P_ADC::W_Current);
DCMotorDriverMCU driver(&mcu, MAL::P_PWM::U_PWM, MAL::P_PWM::V_PWM);
EncoderMCU encoder(&mcu, MAL::P_Encoder::Main_Encoder);

DcMotorController dcMotorController(&driver, &currentSensor, &encoder);
// CommandReceiver commandReceiver(&mcu, MAL::P_UART::Controller, &bldcController);

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

    dcMotorController.init();

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
                                               
 ____       ____       _                
|  _ \  ___|  _ \ _ __(_)_   _____ _ __ 
| | | |/ __| | | | '__| \ \ / / _ \ '__|
| |_| | (__| |_| | |  | |\ V /  __/ |   
|____/ \___|____/|_|  |_| \_/ \___|_|  
                                          
)EOF");

    printf("\x1b[32m[Main Thread]\x1b[39m Initialization Complete\n");
}

void app_main() {
    app_init();

    mcu.gpioSetValue(MAL::P_GPIO::Driver_Power_Switch, true);

    driver.enable(true);
    mcu.waitMs(300);
    dcMotorController.beep(2045, 0.2, 250);
    mcu.waitMs(400);
    dcMotorController.beep(3500, 0.2, 100);
    mcu.waitMs(150);
    dcMotorController.beep(3500, 0.2, 100);
    mcu.waitMs(1000);

    dcMotorController.setMode(DcMotorController::Mode::Stop);

    // bldcController.setEnable(true);
    // bldcController.setMode(BldcController::Mode::CurrentControl);
    // bldcController.setTargetCurrent(0.5, 0);

    // mcu.pwmSetDuty(MAL::P_PWM::U_PWM, 0.95);
    // mcu.pwmSetDuty(MAL::P_PWM::V_PWM, 0.05);

    driver.enable(true);
    dcMotorController.setMode(DcMotorController::Mode::CurrentControl);
    dcMotorController.setEnable(true);
    dcMotorController.setTargetCurrent(0.4);

    while (1) {
        // commandReceiver.update();

        if (feedback_cnt > 5) {
            // commandReceiver.send();
        }

        if (print_cnt > 100) {
            // printf("\x1b[32m[Main Thread]\x1b[39m p_time: %f\n", process_time);
            printf("\x1b[32m[Main Thread]\x1b[39m duty: %f c: %f\n", dcMotorController.getApplyDuty(), dcMotorController.getObservedCurrent());
            print_cnt = 0;
        }
    }
}

void app_interrupt_20us() {  // 50kHz
    mcu.timerSetCnt(MAL::P_TimerCnt::C1, 0);
    dcMotorController.update();

    timer_1ms_cnt++;
    if (timer_1ms_cnt >= 50) {  // 1kHz
        encoder.update1kHz();
        // commandReceiver.cnt++;
        feedback_cnt++;
        print_cnt++;
        mode_cnt++;
        timer_1ms_cnt = 0;
    }

    process_time = float(mcu.timerGetCnt(MAL::P_TimerCnt::C1));
}