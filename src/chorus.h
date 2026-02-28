#ifndef CHORUS_H
#define CHORUS_H

#include <atomic>

extern std::atomic<uint32_t> chorusStep;
extern std::atomic<uint8_t> chorusDepth;

int32_t chorusWave(int32_t drySample);

#endif