#ifndef KNOB_H
#define KNOB_H

#include <stdint.h>
#include <atomic>
#include "STM32FreeRTOS.h"

class Knob {
public:
    Knob(int8_t lower = 0, int8_t upper = 8);
    ~Knob();

    void update(uint8_t inputA, uint8_t inputB);
    int8_t get();

private:
    int8_t minLimit;
    int8_t maxLimit;
    std::atomic<int8_t> rotation;
    uint8_t prevState;
    SemaphoreHandle_t mutex;
};

#endif