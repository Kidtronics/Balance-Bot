
#ifndef SLIDING_WINDOW_FILTER_H
#define SLIDING_WINDOW_FILTER_H

class SlidingWindowFilter {
private: 
    float averageValue;
    float sumSamples;
    int SAMPLE_SIZE;
    int currentSampleSize;

public:
    SlidingWindowFilter(int sampleSize);

    float getAverageValue();

    float addNewSample(float newValue);

    bool numSamplesReachesMax();
};

inline
SlidingWindowFilter::SlidingWindowFilter(int sampleSize) :
    averageValue(0), sumSamples(0), currentSampleSize(0)
{
    SAMPLE_SIZE = sampleSize;
}

inline
float SlidingWindowFilter::getAverageValue() {
    return averageValue;
}

inline
float SlidingWindowFilter::addNewSample(float newValue) {
    if (currentSampleSize == SAMPLE_SIZE) {
        sumSamples -= averageValue;
    }
    else {
        currentSampleSize++;
    }
    sumSamples += newValue;
    averageValue = sumSamples / currentSampleSize;
    return averageValue;
}

inline
bool SlidingWindowFilter::numSamplesReachesMax() {
    return currentSampleSize == SAMPLE_SIZE;
}

#endif