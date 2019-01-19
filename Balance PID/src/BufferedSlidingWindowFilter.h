
#ifndef BUFFERED_SLIDING_WINDOW_FILTER_H
#define BUFFERED_SLIDING_WINDOW_FILTER_H

class BufferedSlidingWindowFilter {
private: 
    // Buffer holding the data in sliding window.
    // This buffer is a circular buffer.
    double* m_buffer;
    // Pointing to the end of buffer.
    int m_bufferIdx;
    // Maximum number of samples this buffer can hold.
    int MAX_SAMPLE_SIZE;
    // Total number of samples in this buffer.
    int m_currentBufferSize;
    // Sum of all samples. This could overflow.
    double m_sumOfSamples;

public:
    BufferedSlidingWindowFilter(double* m_buffer, int sampleSize);

    double getAverageValue();

    void addNewSample(double newValue);

    bool numSamplesReachesMax();
};

#endif