/*
 * baseEncoder.hpp
 *
 *  Created on: Jul 10, 2024
 *      Author: G4T1PR0
 */

#pragma once
#include <stdint.h>

class baseEncoder {
   public:
    void init();
    void update();
    int32_t getCnt();
};