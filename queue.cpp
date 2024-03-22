#include "queue.h"

constexpr float range_threshold{ 0.35f };
constexpr int warm_up_samples{ 30 };

void init_queue(queue* q, int max_size){
    q->size = max_size;
    q->values = (float*)malloc(sizeof(float) * q->size);
    q->num_entries = 0; // we're empty
    q->head = 0;
    q->tail = 0;

    q->previous_range = 0.0f;
    q->bump_counter = 0;

    q->min_index = 0;
    q->max_index = 0;

    q->samples_processed = 0;
    q->bump_detected = false;
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

void apply_bump_detection(queue* q){

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

    if(new_max_index != q->max_index && new_min_index != q->min_index){

        q->max_index = new_max_index;
        q->min_index = new_min_index;

        if(std::abs(q->values[q->max_index] - q->values[q->min_index]) > range_threshold){
            q->bump_counter++;
            q->bump_detected = true;
        }
    }
}

bool enqueue(queue* q, float value){
    if(queue_full(q)){
        // If the queue is full, dequeue the oldest value first
        dequeue(q);
    }
    
    q->values[q->tail] = value;
    q->num_entries++;
    q->tail = (q->tail + 1) % q->size; // wrap around 0 at the end of array


    q->samples_processed++;

    if(q->samples_processed > warm_up_samples){
        apply_bump_detection(q);
    }

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



// -------------------------------------------------------------------------------------

bool push(stack *mystack, int value){

    node *newnode = (node *)malloc(sizeof(node));
    if(newnode == NULL) return false;


    newnode->value = value;
    newnode->next = *mystack;

    *mystack = newnode;

    return true;
}


int pop(stack *mystack){

    if(*mystack == NULL) return STACK_EMPTY;

    int result = (*mystack)->value;
    node *tmp = *mystack;
    *mystack = (*mystack)->next;

    free(tmp);

    return result;
}
