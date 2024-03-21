#include "queue.h"

constexpr float range_threshold{ 0.3f };

void init_queue(queue* q, int max_size){
    q->size = max_size;
    q->values = (float*)malloc(sizeof(float) * q->size);
    q->num_entries = 0; // we're empty
    q->head = 0;
    q->tail = 0;

    q->previous_range = 0.0f;
    q->bump_counter = 0;
}

bool queue_empty(queue* q){
    return(q->num_entries == 0);
}

bool queue_full(queue* q){
    return(q->num_entries == q->size);
}

void queue_destroy(queue* q){
    free(q->values);
}

float dequeue(queue *q){
    float result;

    if(queue_empty(q)){
        return QUEUE_EMPTY;
    }

    result = q->values[q->head];
    q->head = (q->head + 1) % q->size;
    q->num_entries--;

    return result;
}

bool enqueue(queue* q, float value){
    if(queue_full(q)){
        // If the queue is full, dequeue the oldest value first
        dequeue(q);
    }
    
    q->values[q->tail] = value;
    q->num_entries++;
    q->tail = (q->tail + 1) % q->size; // wrap around 0 at the end of array

    // calculating the range

    float max_value = q->values[0];
    float min_value = q->values[0];

    for(int i = 0; i < q->num_entries; ++i){
        if (q->values[i] > max_value) {
            max_value = q->values[i];
        }
        if (q->values[i] < min_value) {
            min_value = q->values[i];
        }
    }

    float new_range = max_value - min_value;

    // comparison of new range with previous range

    float range_difference = new_range - q->previous_range;


    if(range_difference > 0 && range_difference > range_threshold){
        q->bump_counter++;
    }

    q->previous_range = new_range;

    return true;
}



float calculate_mean(queue* q) {
    float sum = 0.0;
    for (int i = 0; i < q->size; i++) {
        sum += q->values[i];
    }
    return sum / q->size;
}

float calculate_std_dev(queue* q, float mean) {
    float sum_of_sq_diff = 0.0;
    for (int i = 0; i < q->size; i++) {
        float diff = q->values[i] - mean;
        sum_of_sq_diff += diff * diff;
    }
    float variance = sum_of_sq_diff / q->size;
    return sqrt(variance);
}

float calculate_variance(queue* q, float mean) {
    float sum_of_sq_diff = 0.0;
    for (int i = 0; i < q->size; i++) {
        float diff = q->values[i] - mean;
        sum_of_sq_diff += diff * diff;
    }
    float variance = sum_of_sq_diff / q->size;
    return variance;
}