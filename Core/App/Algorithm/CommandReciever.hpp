/*
 * CommandReciever.hpp
 *
 *  Created on: Sep 25, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <Algorithm/bldcController.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class CommandReciever {
   public:
    CommandReciever(baseMcuAbstractionLayer* mcu, baseMcuAbstractionLayer::P_UART uart, BldcController* bldcController) {
        _mcu = mcu;
        _uart = uart;
        _bldcController = bldcController;

        _recieveModeTargetValue.d.header[0] = 0xFF;
        _recieveModeTargetValue.d.header[1] = 0xFF;
        _recieveModeTargetValue.d.header[2] = 0xFD;
        _recieveModeTargetValue.d.header[3] = 0x00;

        _recieveModeTargetValue.d.instruction_id = _instruction::SendModeTargetValue;

        _feedbackValue.d.header[0] = 0xFF;
        _feedbackValue.d.header[1] = 0xFF;
        _feedbackValue.d.header[2] = 0xFD;
        _feedbackValue.d.header[3] = 0x00;
    }

    void init() {
    }

    void send() {
    }

    void update() {
        _recieveModeTargetValue.d.header[0] = 0xFF;
        _recieveModeTargetValue.d.header[1] = 0xFF;
        _recieveModeTargetValue.d.header[2] = 0xFD;
        _recieveModeTargetValue.d.header[3] = 0x00;

        _tempRecieveModeTargetValue.d.header[0] = 0xFF;
        _tempRecieveModeTargetValue.d.header[1] = 0xFF;
        _tempRecieveModeTargetValue.d.header[2] = 0xFD;
        _tempRecieveModeTargetValue.d.header[3] = 0x00;

        _feedbackValue.d.header[0] = 0xFF;
        _feedbackValue.d.header[1] = 0xFF;
        _feedbackValue.d.header[2] = 0xFD;
        _feedbackValue.d.header[3] = 0x00;

        unsigned int _rx_size = _mcu->uartGetRxDataSize(_uart);
        if (_rx_size > sizeof(_rx_buffer)) {
            _rx_size = sizeof(_rx_buffer);
        }

        _mcu->uartReadViaBuffer(_uart, _rx_buffer, _rx_size);

        bool _rx_complete = false;

        for (unsigned int i = 0; i < _rx_size; i++) {
            switch (_state) {
                case Header1:
                    if (_rx_buffer[i] == 0xFF) {
                        _state = Header2;
                    } else {
                        // printf("header1\n");
                    }
                    break;

                case Header2:
                    if (_rx_buffer[i] == 0xFF) {
                        _state = Header3;
                    } else {
                        _state = Header1;
                    }
                    break;

                case Header3:
                    if (_rx_buffer[i] == 0xFD) {
                        _state = Header4;
                    } else {
                        _state = Header1;
                    }
                    break;

                case Header4:
                    if (_rx_buffer[i] == 0x00) {
                        _state = CopyToBuffer;
                        _rx_cnt = 4;
                    } else {
                        _state = Header1;
                    }
                    break;

                case CopyToBuffer:
                    _tempRecieveModeTargetValue.b[_rx_cnt] = _rx_buffer[i];
                    _rx_cnt++;

                    if (_rx_cnt >= sizeof(_recieveModeTargetValue_t)) {
                        _state = Header1;
                        _rx_complete = true;
                        _rx_cnt = 0;
                        _recieveModeTargetValue = _tempRecieveModeTargetValue;
                    }
                    break;

                default:
                    break;
            }
        }

        if (_rx_complete) {
            uint32_t rx_data_crc = _mcu->crc32(_recieveModeTargetValue.b, sizeof(_recieveModeTargetValue_t) - sizeof(uint32_t));
            printf("crc: %x ", _recieveModeTargetValue.d.crc);

            printf("crc2 %x \n", rx_data_crc);

            if (_recieveModeTargetValue.d.crc == rx_data_crc) {
                printf("%f\n", _recieveModeTargetValue.d.target);
                // printf("crc ok\n");
                // switch (_recieveModeTargetValue.d.instruction_id) {
                //     case SendModeTargetValue:
                //         _mode = (_Mode)_recieveModeTargetValue.d.mode;
                //         switch (_mode) {
                //             case VoltageControl:
                //                 _target_voltage = _recieveModeTargetValue.d.target;
                //                 break;

                //             case TorqueControl:
                //                 _target_torque = _recieveModeTargetValue.d.target;
                //                 break;

                //             case VelocityControl:
                //                 _target_velocity = _recieveModeTargetValue.d.target;
                //                 break;

                //             default:
                //                 break;
                //         }
                //         break;

                //     default:
                //         break;
                // }

            } else {
                printf("crc fail\n");
            }
            _rx_complete = false;
        }
    }

   private:
    baseMcuAbstractionLayer* _mcu;
    baseMcuAbstractionLayer::P_UART _uart;
    BldcController* _bldcController;

    enum _Mode {
        Stop,
        VoltageControl,
        TorqueControl,
        VelocityControl,
    };

    _Mode _mode;

    float _target_voltage = 0;
    float _current_voltage = 0;

    float _target_torque = 0;
    float _observed_torque = 0;

    float _target_velocity = 0;
    float _observed_velocity = 0;

    uint8_t _rx_buffer[256];

    unsigned int _rx_cnt = 0;

    enum _instruction {
        SendModeTargetValue = 10,
    };

    enum _receivePacketState {
        Header1,
        Header2,
        Header3,
        Header4,
        CopyToBuffer,
    };

    _receivePacketState _state = Header1;

    struct _recieveModeTargetValue_t {
        uint8_t header[4];
        uint8_t instruction_id;
        uint8_t mode;
        float target;
        uint32_t crc;
    } __attribute__((packed));

    struct _feedbackValue_t {
        uint8_t header[4];
        uint8_t mode;
        float voltage;
        float torque;
        float velocity;
        uint32_t crc;
    } __attribute__((packed));

    union _recieveModeTargetValue_union {
        _recieveModeTargetValue_t d;
        uint8_t b[sizeof(_recieveModeTargetValue_t)];
    };

    union _feedbackValue_union {
        _feedbackValue_t d;
        uint8_t b[sizeof(_feedbackValue_t)];
    };

    _recieveModeTargetValue_union _recieveModeTargetValue;
    _recieveModeTargetValue_union _tempRecieveModeTargetValue;
    _feedbackValue_union _feedbackValue;
};