/*
 * CommandReceiver.hpp
 *
 *  Created on: Sep 25, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <Algorithm/bldcController.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

class CommandReceiver {
   public:
    CommandReceiver(baseMcuAbstractionLayer* mcu, baseMcuAbstractionLayer::P_UART uart, BldcController* bldcController) {
        _mcu = mcu;
        _uart = uart;
        _bldcController = bldcController;

        _receiverModeTargetValue.d.header[0] = 0xFF;
        _receiverModeTargetValue.d.header[1] = 0xFF;
        _receiverModeTargetValue.d.header[2] = 0xFD;
        _receiverModeTargetValue.d.header[3] = 0x00;

        _receiverModeTargetValue.d.packet_id = _command_packet_id::SendModeTargetValue;

        _motorStatusFeedback.d.header[0] = 0xFF;
        _motorStatusFeedback.d.header[1] = 0xFF;
        _motorStatusFeedback.d.header[2] = 0xFD;
        _motorStatusFeedback.d.header[3] = 0x00;
    }

    unsigned int cnt = 0;

    void send() {
        _motorStatusFeedback.d.header[0] = 0xFF;
        _motorStatusFeedback.d.header[1] = 0xFF;
        _motorStatusFeedback.d.header[2] = 0xFD;
        _motorStatusFeedback.d.header[3] = 0x00;
        _motorStatusFeedback.d.packet_id = _feedback_packet_id::MotorStatus;

        _motorStatusFeedback.d.mode = _bldcController->getMode();
        _motorStatusFeedback.d.voltage = _bldcController->getApplyQVoltage();
        _motorStatusFeedback.d.torque = _bldcController->getObservedCurrentQ();
        _motorStatusFeedback.d.velocity = _bldcController->getObservedVelocity();

        _motorStatusFeedback.d.crc = _mcu->crc32(_motorStatusFeedback.b, sizeof(_motorStatusFeedback_t) - sizeof(uint32_t));

        _mcu->uartWriteViaBuffer(_uart, _motorStatusFeedback.b, sizeof(_motorStatusFeedback_t));
    }

    void update() {
        _receiverModeTargetValue.d.header[0] = 0xFF;
        _receiverModeTargetValue.d.header[1] = 0xFF;
        _receiverModeTargetValue.d.header[2] = 0xFD;
        _receiverModeTargetValue.d.header[3] = 0x00;

        _tempRecieveModeTargetValue.d.header[0] = 0xFF;
        _tempRecieveModeTargetValue.d.header[1] = 0xFF;
        _tempRecieveModeTargetValue.d.header[2] = 0xFD;
        _tempRecieveModeTargetValue.d.header[3] = 0x00;

        _tempOneShotCommand.d.header[0] = 0xFF;
        _tempOneShotCommand.d.header[1] = 0xFF;
        _tempOneShotCommand.d.header[2] = 0xFD;
        _tempOneShotCommand.d.header[3] = 0x00;

        unsigned int _rx_size = _mcu->uartGetRxDataSize(_uart);
        if (_rx_size > sizeof(_rx_buffer)) {
            _rx_size = sizeof(_rx_buffer);
        }

        _mcu->uartReadViaBuffer(_uart, _rx_buffer, _rx_size);

        bool _rx_complete = false;
        bool _rx_complete_oneshot = false;

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
                        _state = PacketID_Check;
                    } else {
                        _state = Header1;
                    }
                    break;

                case PacketID_Check:
                    switch (_rx_buffer[i]) {
                        case _command_packet_id::SendModeTargetValue:
                            _rx_cnt = 4;
                            _tempRecieveModeTargetValue.b[_rx_cnt] = _rx_buffer[i];
                            _state = CopyToBuffer_ModeTargetValue;
                            _rx_cnt = 5;
                            break;

                        case _command_packet_id::SystemReset:
                            _rx_cnt = 4;
                            _tempOneShotCommand.b[_rx_cnt] = _rx_buffer[i];
                            _state = CopyToBuffer_OneShotCommand;
                            _rx_cnt = 5;
                            break;

                        case _command_packet_id::EnterBootLoader:
                            _rx_cnt = 4;
                            _tempOneShotCommand.b[_rx_cnt] = _rx_buffer[i];
                            _state = CopyToBuffer_OneShotCommand;
                            _rx_cnt = 5;
                            break;

                        default:
                            _state = Header1;
                            _rx_cnt = 0;
                            break;
                    }
                    break;

                case CopyToBuffer_ModeTargetValue:
                    _tempRecieveModeTargetValue.b[_rx_cnt] = _rx_buffer[i];
                    _rx_cnt++;

                    if (_rx_cnt >= sizeof(_receiverModeTargetValue_t)) {
                        _state = Header1;
                        _rx_complete = true;
                        _rx_cnt = 0;
                        _receiverModeTargetValue = _tempRecieveModeTargetValue;
                    }
                    break;

                case CopyToBuffer_OneShotCommand:
                    _tempOneShotCommand.b[_rx_cnt] = _rx_buffer[i];
                    // printf("cnt %d %x ", _rx_cnt, _rx_buffer[i]);
                    _rx_cnt++;

                    if (_rx_cnt >= sizeof(_oneShotCommand_t)) {
                        _state = Header1;
                        _rx_complete_oneshot = true;
                        _rx_cnt = 0;
                        _oneShotCommand = _tempOneShotCommand;
                        printf("\n");
                    }
                    break;

                default:
                    break;
            }
        }

        if (_rx_complete) {
            uint32_t rx_data_crc = _mcu->crc32(_receiverModeTargetValue.b, sizeof(_receiverModeTargetValue_t) - sizeof(uint32_t));
            // printf("crc: %x ", _receiverModeTargetValue.d.crc);

            // printf("crc2 %x \n", rx_data_crc);

            if (_receiverModeTargetValue.d.crc == rx_data_crc) {
                cnt = 0;
                // printf("%f\n", _receiverModeTargetValue.d.target);
                // printf("crc ok\n");
                switch (_receiverModeTargetValue.d.packet_id) {
                    case SendModeTargetValue:
                        _mode = (_Mode)_receiverModeTargetValue.d.mode;
                        // printf("mode %d\n", _mode);
                        switch (_mode) {
                            case Stop:
                                _bldcController->setMode(BldcController::Mode::Stop);
                                break;

                            case VoltageControl:
                                _bldcController->setMode(BldcController::Mode::VoltageControl);
                                _bldcController->setTargetVoltage(_receiverModeTargetValue.d.target, 0);
                                break;

                            case TorqueControl:
                                _bldcController->setMode(BldcController::Mode::CurrentControl);
                                _bldcController->setTargetCurrent(_receiverModeTargetValue.d.target, 0);
                                break;

                            case VelocityControl:
                                _bldcController->setMode(BldcController::Mode::VelocityControl);
                                _bldcController->setTargetVelocity(_receiverModeTargetValue.d.target);
                                break;

                            default:
                                printf("mode error\n");
                                _bldcController->setMode(BldcController::Mode::Stop);
                                break;
                        }
                        break;

                    default:
                        break;
                }

            } else {
                // printf("crc fail\n");
            }
            _rx_complete = false;
        }

        if (_rx_complete_oneshot) {
            // printf("oneshot ");
            for (int i = 0; i < sizeof(_oneShotCommand_t); i++) {
                printf("%x ", _oneShotCommand.b[i]);
            }
            uint32_t rx_data_crc = _mcu->crc32(_oneShotCommand.b, sizeof(_oneShotCommand_union) - sizeof(uint32_t));
            // printf("crc: %x ", _oneShotCommand.d.crc);

            // printf("crc2 %x \n", rx_data_crc);

            if (_oneShotCommand.d.crc == rx_data_crc) {
                cnt = 0;
                switch (_oneShotCommand.d.packet_id) {
                    case SystemReset:
                        _bldcController->setMode(BldcController::Mode::Stop);
                        _mcu->systemReset();
                        break;

                    case EnterBootLoader:
                        _bldcController->setMode(BldcController::Mode::Stop);
                        _mcu->enterBootloader();
                        break;

                    default:
                        break;
                }
            } else {
                // printf("crc fail\n");
            }
            _rx_complete_oneshot = false;
        }
        if (cnt > 100) {
            _bldcController->setEnable(false);
        } else {
            _bldcController->setEnable(true);
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

    uint8_t _rx_buffer[256];

    unsigned int _rx_cnt = 0;

    // Packet ID

    enum _command_packet_id {
        SendModeTargetValue = 10,
        SystemReset = 90,
        EnterBootLoader = 91,
    };

    enum _feedback_packet_id {
        MotorStatus = 20,
    };

    // Packet Process State

    enum _receivePacketState {
        Header1,
        Header2,
        Header3,
        Header4,
        PacketID_Check,
        CopyToBuffer_ModeTargetValue,
        CopyToBuffer_OneShotCommand,
    };

    _receivePacketState _state = Header1;

    // Commmand Protocol

    struct _receiverModeTargetValue_t {
        uint8_t header[4];
        uint8_t packet_id;
        uint8_t mode;
        float target;
        uint32_t crc;
    } __attribute__((packed));

    struct _oneShotCommand_t {
        uint8_t header[4];
        uint8_t packet_id;
        uint8_t arg;
        uint32_t crc;
    } __attribute__((packed));

    // Feedback Protocol

    struct _motorStatusFeedback_t {
        uint8_t header[4];
        uint8_t packet_id;
        uint8_t mode;
        float voltage;
        float torque;
        float velocity;
        uint32_t crc;
    } __attribute__((packed));

    // Packet Buffer Command

    union _receiverModeTargetValue_union {
        _receiverModeTargetValue_t d;
        uint8_t b[sizeof(_receiverModeTargetValue_t)];
    };

    union _oneShotCommand_union {
        _oneShotCommand_t d;
        uint8_t b[sizeof(_oneShotCommand_t)];
    };

    // Packet Buffer Feedback

    union _motorStatusFeedback_union {
        _motorStatusFeedback_t d;
        uint8_t b[sizeof(_motorStatusFeedback_t)];
    };

    _receiverModeTargetValue_union _receiverModeTargetValue;
    _receiverModeTargetValue_union _tempRecieveModeTargetValue;
    _oneShotCommand_union _oneShotCommand;
    _oneShotCommand_union _tempOneShotCommand;

    _motorStatusFeedback_union _motorStatusFeedback;
};