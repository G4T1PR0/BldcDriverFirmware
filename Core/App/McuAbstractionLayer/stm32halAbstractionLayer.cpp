/*
 * stm32halAbstractionLayer.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#include <McuAbstractionLayer/stm32halAbstractionLayer.hpp>
#include <cstring>
#include "adc.h"
// #include "iwdg.h"
#include "cordic.h"
#include "math.h"
#include "tim.h"
#include "usart.h"

struct PeripheralAllocation {
    enum STM_ADC {
        ADC_1,
        ADC_2,
        ADC_3,
        ADC_END
    };

    ADC_HandleTypeDef* ADC_Ins[ADC_END];
    STM_ADC ADC_Connected[MAL::P_ADC::End_A];
    uint8_t ADC_RANK[MAL::P_ADC::End_A];
    uint8_t ADC_INJECTED_RANK[MAL::P_ADC::End_A];

    TIM_HandleTypeDef* PWM_TIM[MAL::P_PWM::End_P];
    uint32_t PWM_CH[MAL::P_PWM::End_P];
    TIM_HandleTypeDef* MASTER_TIM;

    TIM_HandleTypeDef* Encoder_TIM[MAL::P_Encoder::End_E];

    GPIO_TypeDef* GPIO_PORT[MAL::P_GPIO::End_G];
    uint16_t GPIO_PIN[MAL::P_GPIO::End_G];

    UART_HandleTypeDef* UART[MAL::P_UART::End_U];

    TIM_HandleTypeDef* TimerInterrupt_TIM[MAL::P_Interrupt::End_T];
};

static PeripheralAllocation PAL;

stm32halAbstractionLayer::stm32halAbstractionLayer() {
    // ADC
    PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1] = &hadc1;
    PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2] = &hadc2;
    PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3] = &hadc3;

    PAL.ADC_Connected[MAL::P_ADC::U_Current] = PeripheralAllocation::STM_ADC::ADC_2;
    PAL.ADC_INJECTED_RANK[MAL::P_ADC::U_Current] = 1;

    PAL.ADC_Connected[MAL::P_ADC::V_Current] = PeripheralAllocation::STM_ADC::ADC_1;
    PAL.ADC_INJECTED_RANK[MAL::P_ADC::V_Current] = 1;

    PAL.ADC_Connected[MAL::P_ADC::W_Current] = PeripheralAllocation::STM_ADC::ADC_3;
    PAL.ADC_INJECTED_RANK[MAL::P_ADC::W_Current] = 1;
    // PWM

    // Encoder

    // GPIO

    // UART
    PAL.UART[MAL::P_UART::Controller] = &huart3;

    PAL.UART[MAL::P_UART::Debug] = &huart2;

    // Timer Interrupt
    PAL.TimerInterrupt_TIM[MAL::P_Interrupt::T50us] = &htim7;
}

void stm32halAbstractionLayer::init() {
    _initADC();
    _initEncoder();
    _initPWM();
    _initUART();
    _initTimerInterrupt();
}

// ADC
uint16_t stm32halAbstractionLayer::_data[PAL.STM_ADC::ADC_END][3 * ADC_BUFFER_SIZE] = {0};

void stm32halAbstractionLayer::_initADC(void) {
    // if (HAL_ADC_Start_DMA(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1], (uint32_t*)this->_data[PeripheralAllocation::STM_ADC::ADC_1], 2 * ADC_BUFFER_SIZE) !=
    //     HAL_OK) {
    //     Error_Handler();
    // }
    // __HAL_DMA_DISABLE_IT(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1]->DMA_Handle, DMA_IT_TC | DMA_IT_HT);

    // if (HAL_ADC_Start_DMA(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2], (uint32_t*)this->_data[PeripheralAllocation::STM_ADC::ADC_2], 3 * ADC_BUFFER_SIZE) !=
    //     HAL_OK) {
    //     Error_Handler();
    // }
    // __HAL_DMA_DISABLE_IT(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2]->DMA_Handle, DMA_IT_TC | DMA_IT_HT);

    // if (HAL_ADC_Start_DMA(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3], (uint32_t*)this->_data[PeripheralAllocation::STM_ADC::ADC_3], 2 * ADC_BUFFER_SIZE) !=
    //     HAL_OK) {
    //     Error_Handler();
    // }
    // __HAL_DMA_DISABLE_IT(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3]->DMA_Handle, DMA_IT_TC | DMA_IT_HT);

    // HAL_ADCEx_InjectedStart(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1]);
    // HAL_ADCEx_InjectedStart(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2]);
    // HAL_ADCEx_InjectedStart(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3]);
}

uint16_t stm32halAbstractionLayer::adcGetValue(P_ADC p) {
    // if (p != P_ADC::End_A) {
    //     if (p == P_ADC::FL_Current) {
    //         return HAL_ADCEx_InjectedGetValue(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3], 1);
    //     }
    //     if (p == P_ADC::FR_Current) {
    //         return HAL_ADCEx_InjectedGetValue(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_3], 2);
    //     }

    //     if (p == P_ADC::ST_Current) {
    //         return HAL_ADCEx_InjectedGetValue(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2], 1);
    //     }

    //     if (p == P_ADC::RL_Current) {
    //         return HAL_ADCEx_InjectedGetValue(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_2], 2);
    //     }

    //     if (p == P_ADC::RR_Current) {
    //         return HAL_ADCEx_InjectedGetValue(PAL.ADC_Ins[PeripheralAllocation::STM_ADC::ADC_1], 1);
    //     }

    //     return this->_data[PAL.ADC_Connected[p]][PAL.ADC_RANK[PAL.ADC_Connected[p]][p]];
    // }
    return 0;
}

void stm32halAbstractionLayer::adcGetBufferValue(P_ADC p, uint16_t* buffer, uint16_t size) {
    if (p != P_ADC::End_A) {
        // switch (PAL.ADC_Connected[p]) {
        //     case PeripheralAllocation::STM_ADC::ADC_1:
        //         for (int i = 0; i < size; i++) {
        //             buffer[i] = this->_data[PAL.ADC_Connected[p]][i * 2 + PAL.ADC_RANK[PAL.ADC_Connected[p]][p]];
        //         }
        //         break;

        //     case PeripheralAllocation::STM_ADC::ADC_2:
        //         for (int i = 0; i < size; i++) {
        //             buffer[i] = this->_data[PAL.ADC_Connected[p]][i * 3 + PAL.ADC_RANK[PAL.ADC_Connected[p]][p]];
        //         }
        //         break;

        //     case PeripheralAllocation::STM_ADC::ADC_3:
        //         for (int i = 0; i < size; i++) {
        //             buffer[i] = this->_data[PAL.ADC_Connected[p]][i * 2 + PAL.ADC_RANK[PAL.ADC_Connected[p]][p]];
        //         }
        //         break;

        //     default:
        //         break;
        // }
    }
}

// PWM

void stm32halAbstractionLayer::_initPWM() {
    // HAL_TIM_PWM_Start(PAL.PWM_TIM[MAL::P_PWM::FL_PWM], PAL.PWM_CH[MAL::P_PWM::FL_PWM]);

    // HAL_TIM_PWM_Start(PAL.MASTER_TIM, TIM_CHANNEL_1);
}

void stm32halAbstractionLayer::pwmSetDuty(P_PWM p, float duty) {
    if (p != P_PWM::End_P) {
        __HAL_TIM_SET_COMPARE(PAL.PWM_TIM[p], PAL.PWM_CH[p], duty * __HAL_TIM_GET_AUTORELOAD(PAL.PWM_TIM[p]));
    }
}

void stm32halAbstractionLayer::pwmSetFrequency(P_PWM p, uint32_t frequency) {
    if (p != P_PWM::End_P) {
        if (_current_pwm_hz[p] != frequency) {
            _current_pwm_hz[p] = frequency;
            uint32_t apb1_timer_clocks;
            uint32_t apb2_timer_clocks;
            uint32_t timer_clock = 0;

            RCC_ClkInitTypeDef RCC_ClkInitStruct;
            uint32_t pFLatency;
            HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);
            apb1_timer_clocks = HAL_RCC_GetPCLK1Freq();
            apb2_timer_clocks = HAL_RCC_GetPCLK2Freq();
            apb1_timer_clocks *= (RCC_ClkInitStruct.APB1CLKDivider == RCC_HCLK_DIV1) ? 1 : 2;
            apb2_timer_clocks *= (RCC_ClkInitStruct.APB2CLKDivider == RCC_HCLK_DIV1) ? 1 : 2;

            // printf("apb1_timer_clocks: %u\r\n", apb1_timer_clocks);
            // printf("apb2_timer_clocks: %u\r\n", apb2_timer_clocks);

            if ((uint32_t)PAL.PWM_TIM[p]->Instance >= APB2PERIPH_BASE) {
                timer_clock = apb2_timer_clocks;
            } else if ((uint32_t)PAL.PWM_TIM[p]->Instance >= APB1PERIPH_BASE) {
                timer_clock = apb1_timer_clocks;
            }

            for (uint32_t prescaler = 0; prescaler < 65536; prescaler++) {
                for (uint32_t period = 0; period < 65536; period++) {
                    if ((timer_clock / ((prescaler + 1) * (period + 1))) == frequency) {
                        // printf("frequency: %u\r\n", (timer_clock / ((prescaler + 1) * (period + 1))));
                        // printf("timer_clock: %u\r\n", timer_clock);
                        // printf("prescaler: %u\r\n", prescaler + 1);
                        // printf("period: %u\r\n", period + 1);
                        __HAL_TIM_SET_PRESCALER(PAL.PWM_TIM[p], prescaler);
                        __HAL_TIM_SET_AUTORELOAD(PAL.PWM_TIM[p], period);
                        if (__HAL_TIM_GET_COUNTER(PAL.PWM_TIM[p]) >= __HAL_TIM_GET_AUTORELOAD(PAL.PWM_TIM[p])) {
                            PAL.PWM_TIM[p]->Instance->EGR |= TIM_EGR_UG;
                        }
                        __HAL_TIM_SET_CLOCKDIVISION(PAL.PWM_TIM[p], TIM_CLOCKDIVISION_DIV1);
                        return;
                    }  // else if ((timer_clock / ((prescaler + 1) * (period + 1))) > frequency) {
                    //     __HAL_TIM_SET_PRESCALER(PAL.PWM_TIM[p], prescaler);
                    //     __HAL_TIM_SET_AUTORELOAD(PAL.PWM_TIM[p], period);
                    //     if (__HAL_TIM_GET_COUNTER(PAL.PWM_TIM[p]) >= __HAL_TIM_GET_AUTORELOAD(PAL.PWM_TIM[p])) {
                    //         PAL.PWM_TIM[p]->Instance->EGR |= TIM_EGR_UG;
                    //     }
                    //     __HAL_TIM_SET_CLOCKDIVISION(PAL.PWM_TIM[p], TIM_CLOCKDIVISION_DIV1);
                    //     return;
                    // }
                }
            }
        }
    }
}

// Encoder

void stm32halAbstractionLayer::_initEncoder() {
    // HAL_TIM_Encoder_Start(PAL.Encoder_TIM[MAL::P_Encoder::FL_Encoder], TIM_CHANNEL_ALL);
}

void stm32halAbstractionLayer::encoderSetCnt(P_Encoder p, uint32_t cnt) {
    if (p != P_Encoder::End_E) {
        __HAL_TIM_SET_COUNTER(PAL.Encoder_TIM[p], cnt);
    }
}

uint32_t stm32halAbstractionLayer::encoderGetCnt(P_Encoder p) {
    if (p != P_Encoder::End_E) {
        return __HAL_TIM_GET_COUNTER(PAL.Encoder_TIM[p]);
    }
    return 0;
}

// GPIO

void stm32halAbstractionLayer::gpioSetValue(P_GPIO p, bool value) {
    if (p != P_GPIO::End_G) {
        if (value) {
            HAL_GPIO_WritePin(PAL.GPIO_PORT[p], PAL.GPIO_PIN[p], GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(PAL.GPIO_PORT[p], PAL.GPIO_PIN[p], GPIO_PIN_RESET);
        }
    }
}

bool stm32halAbstractionLayer::gpioGetValue(P_GPIO p) {
    if (p != P_GPIO::End_G) {
        return HAL_GPIO_ReadPin(PAL.GPIO_PORT[p], PAL.GPIO_PIN[p]) == GPIO_PIN_SET;
    }
    return false;
}

// UART

RingBuffer<uint8_t, UART_BUFFER_SIZE> stm32halAbstractionLayer::_uartRxBuffer[P_UART::End_U];

void stm32halAbstractionLayer::_initUART() {
    while (HAL_UART_Receive_DMA(PAL.UART[MAL::P_UART::Controller], _uartRxBuffer[MAL::P_UART::Controller].Buffer, UART_BUFFER_SIZE) != HAL_OK) {
    }

    while (HAL_UART_Receive_DMA(PAL.UART[MAL::P_UART::Debug], _uartRxBuffer[MAL::P_UART::Debug].Buffer, UART_BUFFER_SIZE) != HAL_OK) {
    }
}

uint32_t stm32halAbstractionLayer::_uartGetRxBufferDmaWriteAddress(P_UART p) {
    if (p != P_UART::End_U) {
        return (UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(PAL.UART[p]->hdmarx)) % UART_BUFFER_SIZE;
    }
    return 0;
}

void stm32halAbstractionLayer::uartPutChar(P_UART p, uint8_t data) {
    if (p != P_UART::End_U) {
        while (HAL_UART_Transmit_DMA(PAL.UART[p], &data, 1) != HAL_OK) {
        }
    }
}

uint8_t stm32halAbstractionLayer::uartGetChar(P_UART p) {
    uint8_t data = 0;
    if (p != P_UART::End_U) {
        _uartRxBuffer[p].setWritePos(_uartGetRxBufferDmaWriteAddress(p));
        data = _uartRxBuffer[p].pop();
    }
    return data;
}

void stm32halAbstractionLayer::uartWriteViaBuffer(P_UART p, uint8_t* data, uint32_t size) {
    if (p != P_UART::End_U) {
        while (HAL_UART_Transmit_DMA(PAL.UART[p], data, size) != HAL_OK) {
        }
    }
}

void stm32halAbstractionLayer::uartReadViaBuffer(P_UART p, uint8_t* data, uint32_t size) {
    if (p != P_UART::End_U) {
        _uartRxBuffer[p].setWritePos(_uartGetRxBufferDmaWriteAddress(p));
        _uartRxBuffer[p].pop(data, size);
    }

    if (HAL_UART_GetState(PAL.UART[p]) == HAL_UART_STATE_READY) {
        HAL_UART_DMAStop(PAL.UART[p]);
        HAL_UART_Receive_DMA(PAL.UART[p], _uartRxBuffer[p].Buffer, UART_BUFFER_SIZE);
    }
}

uint32_t stm32halAbstractionLayer::uartGetRxDataSize(P_UART p) {
    uint32_t size = 0;
    if (p != P_UART::End_U) {
        _uartRxBuffer[p].setWritePos(_uartGetRxBufferDmaWriteAddress(p));
        size = _uartRxBuffer[p].size();
    }
    return size;
}

// Interrupt

void (*stm32halAbstractionLayer::_timerInterruptCallback[P_Interrupt::End_T])(void);

void stm32halAbstractionLayer::_initTimerInterrupt() {
    HAL_TIM_Base_Start_IT(PAL.TimerInterrupt_TIM[MAL::P_Interrupt::T50us]);
}

void stm32halAbstractionLayer::interruptSetCallback(P_Interrupt p, void (*callback)(void)) {
    if (p != P_Interrupt::End_T) {
        _timerInterruptCallback[p] = callback;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == PAL.TimerInterrupt_TIM[MAL::P_Interrupt::T50us]) {
        if (stm32halAbstractionLayer::_timerInterruptCallback[MAL::P_Interrupt::T50us] != NULL) {
            stm32halAbstractionLayer::_timerInterruptCallback[MAL::P_Interrupt::T50us]();
        }
    }
}

// Wait

void stm32halAbstractionLayer::waitMs(uint32_t ms) {
    HAL_Delay(ms);
}

// Watchdog
void stm32halAbstractionLayer::idwgResetCnt(void) {
    // HAL_IWDG_Refresh(&hiwdg);
}

// Cordic

inline int32_t stm32halAbstractionLayer::RadiansToQ31(float x) {
    // First we scale, then wrap, and finally convert out.
    const float scaled = x / 2 * M_PI;
    // Now we wrap to be from 0 to 1.
    const int32_t i = (int32_t)(scaled);
    float mod = scaled - i;
    if (mod < 0) {
        mod += 1.0f;
    }

    return (int32_t)(((mod > 0.5f) ? (mod - 1.0f) : mod) * 4294967296.0f);
}

inline float stm32halAbstractionLayer::Q31ToRadians(int32_t x) {
    return (float)(x) / 2147483648.0f;
}

float stm32halAbstractionLayer::cordicSin(float a) {
    int32_t input_buffer = RadiansToQ31(a);
    int32_t output_buffer[2] = {0};

    HAL_CORDIC_Calculate(&hcordic, &input_buffer, output_buffer, 1, 0);

    return Q31ToRadians(output_buffer[1]);  // 0 is cos, 1 is sin
}

float stm32halAbstractionLayer::cordicCos(float a) {
    int32_t input_buffer = RadiansToQ31(a);
    int32_t output_buffer[2] = {0};

    HAL_CORDIC_Calculate(&hcordic, &input_buffer, output_buffer, 1, 0);

    return Q31ToRadians(output_buffer[0]);  // 0 is cos, 1 is sin
}

void stm32halAbstractionLayer::cordicSinCos(float a, float* s, float* c) {
    int32_t input_buffer = RadiansToQ31(a);
    int32_t output_buffer[2] = {0};

    HAL_CORDIC_Calculate(&hcordic, &input_buffer, output_buffer, 1, 0);

    *s = Q31ToRadians(output_buffer[1]);  // 0 is cos, 1 is sin
    *c = Q31ToRadians(output_buffer[0]);  // 0 is cos, 1 is sin
}