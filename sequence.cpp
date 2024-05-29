#include "sequence.h"


// Function to calculate the trend of a given window
SequenceType getSequenceType(const std::deque<double>& window) {
    double sum = 0.0;
    for (size_t i = 1; i < window.size(); ++i) {
        sum += window[i] - window[i - 1];
    }
    if (sum > 0.3) return SequenceType::Rising;
    else if (sum < -0.3) return SequenceType::Falling;
    else return SequenceType::Stable;
}


double calculateTrend(const std::deque<double>& window) {
    double sum = 0.0;
    for (size_t i = 1; i < window.size(); ++i) {
        sum += window[i] - window[i - 1];
    }
    return sum;
}

/*
// Function to classify the signal in real-time
SequenceType classifySignalRealTime(const std::deque<double>& signal, size_t windowSize) {
    if (signal.size() < windowSize) return SequenceType::None;
    std::deque<double> window(signal.end() - windowSize, signal.end());
    return calculateTrend(window);
}
*/
