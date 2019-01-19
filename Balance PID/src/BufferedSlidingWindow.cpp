#include "BufferedSlidingWindowFilter.h"

BufferedSlidingWindowFilter::BufferedSlidingWindowFilter(
    double* buffer, 
    int maxSampleSize) 
    :m_bufferIdx(0), m_currentBufferSize(0), m_sumOfSamples(0)
{
    m_buffer = buffer;
    MAX_SAMPLE_SIZE = maxSampleSize;
}

double BufferedSlidingWindowFilter::getAverageValue() {
    return m_sumOfSamples / m_currentBufferSize;
}

void BufferedSlidingWindowFilter::addNewSample(double newValue) {
    if (numSamplesReachesMax()) {
        m_sumOfSamples -= m_buffer[m_bufferIdx];
        m_sumOfSamples += newValue;
        m_buffer[m_bufferIdx] = newValue;
    }
    else {
        m_sumOfSamples += newValue;
        m_buffer[m_bufferIdx] = newValue;
        m_currentBufferSize++;
    }
    m_bufferIdx = (m_bufferIdx + 1) % MAX_SAMPLE_SIZE;
}

bool BufferedSlidingWindowFilter::numSamplesReachesMax() {
    return m_currentBufferSize == MAX_SAMPLE_SIZE;
}