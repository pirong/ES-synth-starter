#ifndef CHORUS_H
#define CHORUS_H

#include <atomic>

extern std::atomic<float> chorusRate;
extern std::atomic<float> chorusDepth;

int32_t chorusWave(int16_t drySample);

#endif