#include "detection.h"
#include "queue.h"


bool combinedToleranceCompare(double x, double y) {
    double maxXYOne = std::max({1.0, std::fabs(x), std::fabs(y)});
    return std::fabs(x - y) <= std::numeric_limits<double>::epsilon() * maxXYOne;
}

void init_detection(detection* detect){
    detect->samples_processed = 0;
    detect->bump_detected = false;
    detect->cooldown_counter = 0; // Initialize cooldown_counter
    detect->bump_counter = 0;
}

void apply_detection(detection* detect, std::deque<double>& data){

    detect->samples_processed++;

    if(detect->samples_processed > warm_up_samples){
        
        // Decrement the cooldown counter
        if (detect->cooldown_counter > 0) {
            detect->cooldown_counter--;
            return; // Skip the rest of the function if we're in cooldown
        }

        // Find the maximum and minimum values in the deque
        auto max_value_iter = std::max_element(data.begin(), data.end());
        auto min_value_iter = std::min_element(data.begin(), data.end());

        double max_value = *max_value_iter;
        double min_value = *min_value_iter;

        // Calculate the new range
        // double new_range = max_value - min_value;

        if(std::abs(max_value - min_value) > range_threshold){
            detect->bump_counter++;
            detect->bump_detected = true;

            // Set the cooldown counter to a value that represents the cooldown period
            detect->cooldown_counter = cooldown_samples;
        }
    }
}