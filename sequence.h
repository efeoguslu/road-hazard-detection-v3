#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <deque>
#include <cstddef>
#include <string>

enum class SequenceType {
    Rising,
    Falling,
    Stable,
    None
};




SequenceType calculateTrend(const std::deque<double>& window);
SequenceType classifySignalRealtime(const std::deque<double>& signal, size_t windowSize);

#endif
