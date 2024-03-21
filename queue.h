#ifndef QUEUE_H
#define QUEUE_H


#include <cstdlib>
#include <climits>
#include <cmath>



// detect when queue is empty:
#define QUEUE_EMPTY INT_MIN

typedef struct{
    int *values;
    int head, tail, num_entries, size;
} queue;



void init_queue(queue* q, int max_size);
bool queue_empty(queue* q);
bool queue_full(queue* q);
void queue_destroy(queue* q);
int dequeue(queue *q);
bool enqueue(queue* q, int value);

double calculate_mean(queue* q);
double calculate_std_dev(queue* q, double mean);


#endif // QUEUE_H