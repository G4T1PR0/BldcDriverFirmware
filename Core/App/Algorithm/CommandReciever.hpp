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
    }

    void init() {
    }

    void update() {
    }

   private:
    baseMcuAbstractionLayer* _mcu;
    baseMcuAbstractionLayer::P_UART _uart;
    BldcController* _bldcController;
};