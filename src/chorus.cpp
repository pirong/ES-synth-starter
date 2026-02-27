#include "main.h"
#include "knob.h"

#include <atomic>

#define CHORUS_MAX_DELAY_MS 25
#define CHORUS_BUFFER_SIZE int((SAMPLE_RATE * CHORUS_MAX_DELAY_MS) / 1000)

int16_t chorusBuffer[CHORUS_BUFFER_SIZE];
uint32_t chorusWriteIndex = 0;

extern Knob knob1;
extern Knob knob2;

// LFO
float chorusPhase = 0.0f;
std::atomic<float> chorusRate;       // Hz
std::atomic<float> chorusDepth;      // samples
float chorusBaseDelay = 25.0f;  // samples (~1.1ms at 22k)

inline int32_t chorusWave(int16_t drySample) {

  // --- LFO ---
  float localChorusRate = knob1.get() / 2.0f;  // Between 0-4 Hz
  chorusRate = localChorusRate;

  float localChorusDepth = knob2.get() * 2.0f; // Max depth of 10 samples
  chorusDepth = localChorusDepth;
  
  chorusPhase += (2.0f * PI * chorusRate) / SAMPLE_RATE;
  if (chorusPhase > 2.0f * PI)
    chorusPhase -= 2.0f * PI;

  float lfo = sinf(chorusPhase);  // -1..1
  float modulatedDelay = chorusBaseDelay + (lfo * chorusDepth);

  // Read position
  int32_t readIndex = chorusWriteIndex - (int32_t)modulatedDelay;
  if (readIndex < 0)
    readIndex += CHORUS_BUFFER_SIZE;

  int16_t delayedSample = chorusBuffer[readIndex];

  // Write current sample
  chorusBuffer[chorusWriteIndex] = drySample;

  chorusWriteIndex++;
  if (chorusWriteIndex >= CHORUS_BUFFER_SIZE)
    chorusWriteIndex = 0;

  // Mix 50/50
  return (drySample >> 1) + (delayedSample >> 1);
}