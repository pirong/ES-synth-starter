#include "main.h"
#include "knob.h"

#include <atomic>
#include <stdint.h>

extern Knob knob1;           // Tremolo speed
extern Knob knob2;           // Tremolo depth
extern uint8_t sineTable[256];

std::atomic<uint32_t> tremoloStep;   // phase increment
std::atomic<uint8_t> tremoloDepth;   // 0–max amplitude scaling

static uint32_t tremoloPhase = 0;

// Tremolo depth scaling factor (1 = full amplitude)
#define TREMOLO_MAX_GAIN 128

int32_t tremoloWave(int32_t drySample){
    // ---- LFO ----
    uint8_t localTremeloDepth = tremoloDepth.load(std::memory_order_relaxed);
    tremoloPhase += tremoloStep.load(std::memory_order_relaxed);

    // Top 8 bits select sine index
    uint8_t index = tremoloPhase >> 24;

    // LFO value 0–255
    uint8_t lfo = sineTable[index];

    // Convert to signed -128..127
    int16_t lfoSigned = (int16_t)lfo - 128;

    // Map LFO (-128..127) to 0..TREMOLO_MAX_GAIN
    // Scale by depth
    uint16_t gain = TREMOLO_MAX_GAIN - localTremeloDepth + ((int32_t)lfoSigned * localTremeloDepth >> 7);

    // Apply gain to sample (0..2*TREMOLO_MAX_GAIN) then scale back
    int32_t modulatedSample = (drySample * gain) >> 7;

    return modulatedSample;
}