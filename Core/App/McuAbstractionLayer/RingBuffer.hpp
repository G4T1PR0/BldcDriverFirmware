/*
 * RingBuffer.hpp
 * Created on: 2024 2/17
 * Author: G4T1PR0
 */

#ifndef __RING_BUFFER_LIB__
#define __RING_BUFFER_LIB__

template <typename T, int capacity>
class RingBuffer {
   public:
    RingBuffer() {
        _capacity = capacity;
        _write_pos = 0;
        _read_pos = 0;
        for (int i = 0; i < _capacity; i++) {
            Buffer[i] = 0;
        }
    }

    T Buffer[capacity];

    void push(T value) {
        Buffer[_write_pos] = value;
        _write_pos = (_write_pos + 1) % _capacity;
    }

    T pop() {
        T value = 0;
        if (_write_pos == _read_pos) {
            return 0;
        } else {
            value = Buffer[_read_pos];
            _read_pos = (_read_pos + 1) % _capacity;
        }

        return value;
    }

    void pop(T* dest, int count) {
        for (int i = 0; i < count; i++) {
            dest[i] = pop();
        }
    }

    int size() {
        if (_write_pos >= _read_pos) {
            return _write_pos - _read_pos;
        } else {
            return _capacity - _read_pos + _write_pos;
        }
    }

    bool empty() {
        return _write_pos == _read_pos;
    }

    T get(int index) {
        int i = (_read_pos + index) % _capacity;
        return Buffer[i];
    }

    void clear() {
        _write_pos = 0;
        _read_pos = 0;
    }

    void set(int index, T value) {
        int i = (_read_pos + index) % _capacity;
        Buffer[i] = value;
    }

    void setWritePos(int index) {
        _write_pos = index;
    }

    void setReadPos(int index) {
        _read_pos = index;
    }

    T operator[](int index) {
        return get(index);
    }

    void operator=(RingBuffer<T, capacity> other) {
        _capacity = other._capacity;
        _write_pos = other._write_pos;
        _read_pos = other._read_pos;
        for (int i = 0; i < _capacity; i++) {
            Buffer[i] = other.Buffer[i];
        }
    }

   private:
    int _capacity;
    int _write_pos;
    int _read_pos;
};

#endif