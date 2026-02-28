#ifndef TREMOLO_H
#define TREMOLO_H

#include <atomic>

extern std::atomic<uint32_t> tremoloStep;
extern std::atomic<uint8_t> tremoloDepth;

int32_t tremoloWave(int32_t drySample);

#endif