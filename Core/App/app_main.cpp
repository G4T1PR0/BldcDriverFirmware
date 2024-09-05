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

// CurrentSensorMCU currentSensor(&mcu, MAL::P_ADC::U_Current, MAL::P_ADC::V_Current, MAL::P_ADC::W_Current);
// BLDCDriverMCU Driver(&mcu, MAL::P_PWM::U_PWM, MAL::P_PWM::V_PWM, MAL::P_PWM::W_PWM);
// EncoderMCU encoder(&mcu, MAL::P_Encoder::Main_Encoder);

// AngleProcessor angleProcessor(&encoder);
// CurrentProcessor currentProcessor(&mcu, &currentSensor);
// ModulationProcessor modulationProcessor(&mcu, &Driver);
// BldcController bldcController(&angleProcessor, &currentProcessor, &modulationProcessor);

void app_interrupt_50us();

void app_init() {
    mcu.init();

    // currentSensor.init();

    mcu.interruptSetCallback(MAL::P_Interrupt::T50us, app_interrupt_50us);

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
    printf("Hello World!\n");

    while (1) {
    }
}

void app_interrupt_50us() {
    // printf("Interrupt 1ms\n");
}