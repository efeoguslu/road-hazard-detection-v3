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

void apply_detection(detection* detect, queue* q){

    detect->samples_processed++;

    if(detect->samples_processed > warm_up_samples){
        
        // Decrement the cooldown counter
        if (detect->cooldown_counter > 0) {
            detect->cooldown_counter--;
            return; // Skip the rest of the function if we're in cooldown
        }

        float max_value = q->values[0];
        float min_value = q->values[0];

        int new_max_index = 0;
        int new_min_index = 0;

        for(int i = 0; i < q->num_entries; ++i){
            if (q->values[i] > max_value) {
                max_value = q->values[i];
                new_max_index = i;
            }
            if (q->values[i] < min_value) {
                min_value = q->values[i];
                new_min_index = i;
            }
        }

        float new_range = max_value - min_value;

        if(!combinedToleranceCompare(new_range, q->previous_range)){

            if(new_max_index != q->max_index && new_min_index != q->min_index){

            q->max_index = new_max_index;
            q->min_index = new_min_index;

                if(std::abs(q->values[q->max_index] - q->values[q->min_index]) > range_threshold){
                    detect->bump_counter++;
                    detect->bump_detected = true;

                    // Set the cooldown counter to a value that represents the cooldown period
                    detect->cooldown_counter = cooldown_samples;
                }

            }

        }

        // Update the previous range
        q->previous_range = new_range;
        }
}