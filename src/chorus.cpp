#include "main.h"
#include "knob.h"

#include <atomic>

#define CHORUS_MAX_DELAY_MS 25
#define CHORUS_BUFFER_SIZE int((SAMPLE_RATE * CHORUS_MAX_DELAY_MS) / 1000)

int32_t chorusBuffer[CHORUS_BUFFER_SIZE];
uint32_t chorusWriteIndex = 0;

extern Knob knob1;
extern Knob knob2;
extern uint8_t sineTable[256];

std::atomic<uint32_t> chorusStep;       // phase increment
std::atomic<uint8_t> chorusDepth;      // samples

static uint32_t chorusPhase = 0;
uint8_t  chorusBaseDelay = 25;

int32_t chorusWave(int32_t drySample) {

    // ---- LFO ----
    chorusPhase += chorusStep.load(std::memory_order_relaxed);

    // Top 8 bits select sine index
    uint8_t index = chorusPhase >> 24;

    // 0–255
    uint8_t lfo = sineTable[index];

    // Convert to signed -128..127
    int16_t lfoSigned = (int16_t)lfo - 128;

    // Scale by depth (samples)
    int32_t modulatedDelay = chorusBaseDelay + ((int32_t)lfoSigned * chorusDepth.load(std::memory_order_relaxed) >> 7);

    // ---- Delay read ----
    int32_t readIndex = chorusWriteIndex - modulatedDelay;
    if (readIndex < 0)
        readIndex += CHORUS_BUFFER_SIZE;

    int32_t delayedSample = chorusBuffer[readIndex];

    // ---- Write ----
    chorusBuffer[chorusWriteIndex] = drySample;

    if (++chorusWriteIndex >= CHORUS_BUFFER_SIZE)
        chorusWriteIndex = 0;

    // 50/50 mix
    return (drySample + delayedSample) >> 1;
}