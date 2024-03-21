#include "queue.h"




void init_queue(queue* q, int max_size){
    q->size = max_size;
    q->values = (int*)malloc(sizeof(int) * q->size);
    q->num_entries = 0; // we're empty
    q->head = 0;
    q->tail = 0;
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

int dequeue(queue *q){
    int result;

    if(queue_empty(q)){
        return QUEUE_EMPTY;
    }

    result = q->values[q->head];
    q->head = (q->head + 1) % q->size;
    q->num_entries--;

    return result;
}

bool enqueue(queue* q, int value){
    if(queue_full(q)){
        // If the queue is full, dequeue the oldest value first
        dequeue(q);
    }
    
    q->values[q->tail] = value;
    q->num_entries++;
    q->tail = (q->tail + 1) % q->size; // wrap around 0 at the end of array

    return true;
}



double calculate_mean(queue* q) {
    double sum = 0.0;
    for (int i = 0; i < q->size; i++) {
        sum += q->values[i];
    }
    return sum / q->size;
}

double calculate_std_dev(queue* q, double mean) {
    double sum_of_sq_diff = 0.0;
    for (int i = 0; i < q->size; i++) {
        double diff = q->values[i] - mean;
        sum_of_sq_diff += diff * diff;
    }
    double variance = sum_of_sq_diff / q->size;
    return sqrt(variance);
}