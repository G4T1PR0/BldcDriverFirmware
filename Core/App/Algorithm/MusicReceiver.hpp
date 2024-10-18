/*
 * MusicReceiver.hpp
 *
 *  Created on: Oct 18, 2024
 *      Author: G4T1PR0
 */

#pragma once

#include <Algorithm/BldcController.hpp>
#include <McuAbstractionLayer/RingBuffer.hpp>
#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

#define MUSIC_RECEIVER_BUFFER_SIZE 60000

class MusicReceiver {
   public:
    MusicReceiver(baseMcuAbstractionLayer* mcu, MAL::P_UART uart, BldcController* bldcController) {
        _mcu = mcu;
        _uart = uart;
        _bldc = bldcController;
    }

    void update() {
        unsigned int rx_size = _mcu->uartGetRxDataSize(_uart);
        // printf("rx_size: %d\n", rx_size);
        if (rx_size > 0) {
            if (rx_size > 1024) {
                rx_size = 1024;
            }

            for (unsigned int i = 0; i < rx_size; i++) {
                uint8_t data = _mcu->uartGetChar(_uart);

                switch (_rx_state) {
                    case Header1:
                        if (data == 0xFF) {
                            _rx_state = Header2;
                            // printf("Header2\n");
                        } else {
                            _rx_state = Header1;
                            // printf("Header1\n");
                        }
                        break;

                    case Header2:
                        if (data == 0xFD) {
                            _rx_state = Header3;
                        } else {
                            _rx_state = Header1;
                        }
                        break;

                    case Header3:
                        if (data == 0x00) {
                            _rx_state = Data;
                            _rx_cnt = 0;
                        } else {
                            _rx_state = Header1;
                        }
                        break;

                    case Data:
                        if (_rx_cnt == 0) {
                            _temp_rx = data << 8;
                            _rx_cnt = 1;
                        } else if (_rx_cnt == 1) {
                            _temp_rx = _temp_rx | data;
                            _rx_cnt = 0;
                            // printf("%d\n", _temp_rx);
                            _rx_state = Header1;
                            _buffer.push(_temp_rx);
                        }
                        break;
                }
            }
        }

        if (_buffer.size() >= MUSIC_RECEIVER_BUFFER_SIZE * 0.8) {
            printf("x\n");
            _play = true;
        }

        if (_buffer.size() <= MUSIC_RECEIVER_BUFFER_SIZE * 0.6 && _play) {
            printf("q\n");
        }

        if (_buffer.size() == 0 && _play) {
            printf("w\n");
            _play = false;
        }

        // for (int i = 0; i < _buffer.size(); i++) {
        //     printf("%d\n", _buffer.pop());
        // }

        // printf("size: %d\n", _buffer.size());
    }

    void updateBldc() {
        if (_play) {
            float amplitude = _buffer.pop() * 0.00001;

            _mode_cnt++;
            switch (_mode) {
                case 0:
                    if (_mode_cnt > 250 * 100) {
                        _mode = 1;
                        _mode_cnt = 0;
                    };
                    break;

                case 1:
                    amplitude += 0.07;
                    if (_mode_cnt > 250 * 100) {
                        _mode = 2;
                        _mode_cnt = 0;
                    }
                    break;

                case 2:
                    if (_mode_cnt > 250 * 100) {
                        _mode = 3;
                        _mode_cnt = 0;
                    }
                    break;

                case 3:
                    amplitude -= 0.07;
                    if (_mode_cnt > 250 * 100) {
                        _mode = 0;
                        _mode_cnt = 0;
                    }
                    break;

                default:
                    _mode = 0;
                    break;
            }

            _bldc->setTargetCurrent(amplitude, 0);
        } else {
            _bldc->setTargetCurrent(0, 0);
        }
    }

   private:
    RingBuffer<int16_t, MUSIC_RECEIVER_BUFFER_SIZE> _buffer;
    baseMcuAbstractionLayer* _mcu;
    BldcController* _bldc;

    enum RX_STATE {
        Header1,
        Header2,
        Header3,
        Data,
    };

    int16_t _temp_rx;

    RX_STATE _rx_state = Header1;

    unsigned int _rx_cnt;

    bool _play = false;

    int _mode = 0;
    unsigned int _mode_cnt = 0;

    MAL::P_UART _uart;
};