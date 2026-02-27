#include "knob.h"
#include <algorithm>

Knob::Knob(int8_t lower, int8_t upper)
    : minLimit(lower), maxLimit(upper), rotation(0), prevState(0)
{
    mutex = xSemaphoreCreateMutex();
}

Knob::~Knob() {
    if (mutex) {
        vSemaphoreDelete(mutex);
    }
}

void Knob::update(uint8_t inputA, uint8_t inputB) {
    uint8_t currState = (inputB << 1) | inputA;
    int8_t change = 0;

    if ((prevState == 0b00 && currState == 0b01) ||
        (prevState == 0b11 && currState == 0b10)) {
        change = 1;
    }
    else if ((prevState == 0b01 && currState == 0b00) ||
             (prevState == 0b10 && currState == 0b11)) {
        change = -1;
    }

    xSemaphoreTake(mutex, portMAX_DELAY);

    if (change != 0) {
        rotation += change;
        int8_t value = rotation.load(std::memory_order_relaxed);
        value = std::max(minLimit, std::min(maxLimit, value));
        rotation.store(value, std::memory_order_relaxed);
    }

    prevState = currState;

    xSemaphoreGive(mutex);
}

int8_t Knob::get() {
    return rotation.load(std::memory_order_relaxed);
}