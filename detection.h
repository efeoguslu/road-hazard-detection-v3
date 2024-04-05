#ifndef DETECTION_H
#define DETECTION_H

#include <algorithm> // For std::max
#include "queue.h"

constexpr int cooldown_samples{ 20 };
constexpr float range_threshold{ 0.35f };
constexpr int warm_up_samples{ 30 };

typedef struct{
    int samples_processed;
    bool bump_detected;
    int cooldown_counter; // Track the number of samples to ignore after a bump
    int bump_counter;

} detection;


void init_detection(detection* detect);
void apply_detection(detection* detect, queue* q);


#endif // DETECTION_H