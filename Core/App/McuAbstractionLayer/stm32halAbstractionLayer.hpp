/*
 * stm32halAbstractionLayer.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#ifndef APP_DEVICES_STM32HALABSTRACTIONLAYER_STM32HALABSTRACTIONLAYER_HPP_
#define APP_DEVICES_STM32HALABSTRACTIONLAYER_STM32HALABSTRACTIONLAYER_HPP_

#include <McuAbstractionLayer/RingBuffer.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

#define STM32_MAL_UART_BUFFER_SIZE 1024
#define STM32_MAL_ADC_BUFFER_SIZE 50
#define STM32_MAL_MCU_MODEL_NUMBER H7x

#if defined(__ICCARM__)
#define DMA_BUFFER \
    _Pragma("location=\".dma_buffer\"")
#else
#define DMA_BUFFER \
    __attribute__((section(".dma_buffer")))
#endif

class stm32halAbstractionLayer : public baseMcuAbstractionLayer {
   public:
    stm32halAbstractionLayer();
    virtual void init(void);

    // ADC
    virtual uint16_t adcGetValue(P_ADC p);
    virtual void adcGetBufferValue(P_ADC p, uint16_t* buffer, uint16_t size);

    // PWM
    virtual void pwmSetDuty(P_PWM p, float duty);
    virtual void pwmSetFrequency(P_PWM p, uint32_t frequency);

    // Encoder
    virtual void encoderSetCnt(P_Encoder p, uint32_t cnt);
    virtual uint32_t encoderGetCnt(P_Encoder p);

    // GPIO
    virtual void gpioSetValue(P_GPIO p, bool value);
    virtual bool gpioGetValue(P_GPIO p);

    // UART
    virtual void uartPutChar(P_UART p, uint8_t data);
    virtual uint8_t uartGetChar(P_UART p);

    virtual void uartWriteViaBuffer(P_UART p, uint8_t* data, uint32_t size);
    virtual void uartReadViaBuffer(P_UART p, uint8_t* data, uint32_t size);
    virtual uint32_t uartGetRxDataSize(P_UART p);

    // Interrupt
    virtual void interruptSetCallback(P_Interrupt p, void (*callback)(void));
    static void (*_timerInterruptCallback[P_Interrupt::End_T])(void);

    // Wait
    virtual void waitMs(uint32_t ms);

    // Watchdog
    virtual void idwgResetCnt(void);

    // Cordic
    virtual float cordicSin(float a);
    virtual float cordicCos(float a);
    virtual void cordicSinCos(float a, float* s, float* c);

    // Timer Counter
    virtual void timerSetCnt(P_TimerCnt p, uint32_t cnt);
    virtual uint32_t timerGetCnt(P_TimerCnt p);

    // Bootloader
    virtual void enterBootloader(void);

    // System Reset
    virtual void systemReset(void);

    // CRC
    virtual uint32_t crc32(uint8_t* data, uint32_t size);

   private:
    // ADC
    void _initADC();

    static uint16_t _data[3][3 * STM32_MAL_ADC_BUFFER_SIZE];

    // Timer PWM
    void _initPWM();
    unsigned int _current_pwm_hz[P_PWM::End_P] = {0};

    // Timer Encoder
    void _initEncoder();

    // UART
    void _initUART();
    uint32_t _uartGetRxBufferDmaWriteAddress(P_UART p);
    DMA_BUFFER static RingBuffer<uint8_t, STM32_MAL_UART_BUFFER_SIZE> _uartRxBuffer[P_UART::End_U];
    DMA_BUFFER static uint8_t _uartTxBuffer[P_UART::End_U][STM32_MAL_UART_BUFFER_SIZE];
    uint32_t _uartRxBufferReadAddress[P_UART::End_U] = {0};

    // Interrupt
    void _initTimerInterrupt();

    // Cordic
    int32_t RadiansToQ31(float x);
    float Q31ToRadians(int32_t x);

    // Timer Counter
    void _initTimerCounter();

    // Bootloader
    enum Model { C0,
                 F030x8,
                 F030xC,
                 F03xx,
                 F05,
                 F07,
                 F09,
                 F10xx,
                 F105,
                 F107,
                 F10XL,
                 F2,
                 F3,
                 F4,
                 F7,
                 G0,
                 G4,
                 H503,
                 H563,
                 H573,
                 H7x,
                 H7A,
                 H7B,
                 L0,
                 L1,
                 L4,
                 L5,
                 WBA,
                 WBX,
                 WL,
                 U5 };

    Model _model = Model::STM32_MAL_MCU_MODEL_NUMBER;
};

#endif /* APP_DEVICES_STM32HALABSTRACTIONLAYER_STM32HALABSTRACTIONLAYER_HPP_ */