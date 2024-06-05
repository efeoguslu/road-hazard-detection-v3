#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <deque>
#include <cstddef>
#include <string>

enum class SequenceType {
    Rising,  // 0
    Falling, // 1
    Stable,  // 2
    None
};

enum class EventType{
    Bump,
    Pothole,
    Stable,
    None
};

SequenceType getSequenceType(const std::deque<double>& window);
double calculateTrend(const std::deque<double>& window);
// SequenceType classifySignalRealTime(const std::deque<double>& signal, size_t windowSize);

#endif
