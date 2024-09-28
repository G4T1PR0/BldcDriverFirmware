/*
 * baseMcuAbstractionLayer.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_BASEMCUABSTRACTIONLAYER_MCUABSTRACTIONLAYER_HPP_
#define APP_DEVICES_BASEMCUABSTRACTIONLAYER_MCUABSTRACTIONLAYER_HPP_

#include <stdint.h>

class baseMcuAbstractionLayer {
   public:
    enum P_ADC {
        U_Current,
        V_Current,
        W_Current,
        End_A,
    };

    enum P_PWM {
        U_PWM,
        V_PWM,
        W_PWM,
        End_P,
    };

    enum P_Encoder {
        Main_Encoder,
        End_E,
    };

    enum P_GPIO {
        Driver_Power_Switch,
        End_G,
    };

    enum P_UART {
        Controller,
        Debug,
        End_U,
    };

    enum P_Interrupt {
        T20us,
        End_T
    };

    enum P_TimerCnt {
        C1,
        End_C
    };

    virtual void init(void) = 0;

    // ADC
    virtual uint16_t adcGetValue(P_ADC p) = 0;
    virtual void adcGetBufferValue(P_ADC p, uint16_t* buffer, uint16_t size) = 0;

    // PWM
    virtual void pwmSetDuty(P_PWM p, float duty) = 0;
    virtual void pwmSetFrequency(P_PWM p, uint32_t frequency) = 0;

    // Encoder
    virtual void encoderSetCnt(P_Encoder p, uint32_t cnt) = 0;
    virtual uint32_t encoderGetCnt(P_Encoder p) = 0;

    // GPIO
    virtual void gpioSetValue(P_GPIO p, bool value) = 0;
    virtual bool gpioGetValue(P_GPIO p) = 0;

    // UART
    virtual void uartPutChar(P_UART p, uint8_t data) = 0;
    virtual uint8_t uartGetChar(P_UART p) = 0;
    virtual void uartWriteViaBuffer(P_UART p, uint8_t* data, uint32_t size) = 0;
    virtual void uartReadViaBuffer(P_UART p, uint8_t* data, uint32_t size) = 0;
    virtual uint32_t uartGetRxDataSize(P_UART p) = 0;

    // Interrupt
    virtual void interruptSetCallback(P_Interrupt p, void (*callback)(void)) = 0;

    // Wait
    virtual void waitMs(uint32_t ms) = 0;

    // Watchdog
    virtual void idwgResetCnt(void) = 0;

    // Cordic
    virtual float cordicSin(float a) = 0;
    virtual float cordicCos(float a) = 0;
    virtual void cordicSinCos(float a, float* s, float* c) = 0;

    // Timer Counter
    virtual void timerSetCnt(P_TimerCnt p, uint32_t cnt) = 0;
    virtual uint32_t timerGetCnt(P_TimerCnt p) = 0;

    // Bootloader
    virtual void enterBootloader(void) = 0;

    // CRC
    virtual uint32_t crc32(uint8_t* data, uint32_t size) = 0;
};

typedef baseMcuAbstractionLayer MAL;

#endif /* APP_DEVICES_BASEMCUABSTRACTIONLAYER_MCUABSTRACTIONLAYER_HPP_ */