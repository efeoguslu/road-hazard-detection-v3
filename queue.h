#ifndef QUEUE_H
#define QUEUE_H


#include <cstdlib>
#include <climits>
#include <cmath>



// detect when queue is empty:
#define QUEUE_EMPTY INT_MIN

typedef struct{
    float *values;
    int head, tail, num_entries, size;

    float previous_range;
    int bump_counter;
} queue;



void init_queue(queue* q, int max_size);
bool queue_empty(queue* q);
bool queue_full(queue* q);
void queue_destroy(queue* q);
float dequeue(queue *q);
bool enqueue(queue* q, float value);

float calculate_mean(queue* q);
float calculate_std_dev(queue* q, float mean);
float calculate_variance(queue* q, float mean);


#endif // QUEUE_H