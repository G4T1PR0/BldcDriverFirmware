/*
 *  MovingAverageFilter.hpp
 *
 *  Created on: 2024/3/3
 *
 *  Author: G4T1PR0
 */

#ifndef __MOVING_AVERAGE_FILTER_LIB__
#define __MOVING_AVERAGE_FILTER_LIB__

template <typename T, int size>

class MovingAverageFilter {
   public:
    MovingAverageFilter() {
    }

    T Array[size];

    void init() {
        for (int i = 0; i < size; i++) {
            Array[i] = 0;
        }
    }

    void push(T value) {
        Array[_index] = value;
        _index = (_index + 1) % size;
        average();
    }

    void average() {
        _sum = 0;
        for (int i = 0; i < size; i++) {
            _sum += Array[i];
        }
        _average = _sum / size;
    }

    T get() {
        return _average;
    }

   private:
    unsigned int _index = 0;
    T _sum;
    T _average;
};

#endif /* __MOVING_AVERAGE_FILTER_LIB__ */
