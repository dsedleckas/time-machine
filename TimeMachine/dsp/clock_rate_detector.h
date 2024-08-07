#include "daisysp.h"

#ifndef CLOCK_RATE_DETECTOR_H_
#define CLOCK_RATE_DETECTOR_H_

class ClockRateDetector {
public:
  int samplesSinceLastClock;
  int lastIntervalInSamples;
  bool lastVal;
  float sampleRate;
  ClockRateDetector() {
    samplesSinceLastClock = 0;
    lastIntervalInSamples = 0;
    lastVal = false;
  }
  void Init(int sr) { sampleRate = sr; }
  
  bool isStale() {
    return samplesSinceLastClock > sampleRate * 2;
  }
  float GetInterval() {
    float interval = lastIntervalInSamples / sampleRate;
    return isStale() ? 0.0 : interval;
  }
  void Process(bool triggered) {
    if(triggered && lastVal != triggered) {
      if(isStale()) {
        lastIntervalInSamples = samplesSinceLastClock;
      } else {
        lastIntervalInSamples = (lastIntervalInSamples + samplesSinceLastClock) * 0.5;
      }
      samplesSinceLastClock = 0;
    } else {
      samplesSinceLastClock++;
    }
    lastVal = triggered;
  }
};

#endif // CLOCK_RATE_DETECTOR_H_