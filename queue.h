#ifndef QUEUE_H
#define QUEUE_H


#include <cstdlib>
#include <climits>
#include <cmath>
#include <unistd.h>



// detect when queue/stack is empty:
#define QUEUE_EMPTY (INT_MIN)
#define STACK_EMPTY (INT_MIN)


typedef struct{
    float *values;
    int head, tail, num_entries, size;

    float previous_range;
    int bump_counter;
    int min_index;
    int max_index;
    int samples_processed;
    bool bump_detected;
} queue;



typedef struct node{
    int value;
    struct node *next;
} node;


typedef node * stack;



void init_queue(queue* q, int max_size);
bool queue_empty(queue* q);
bool queue_full(queue* q);
void queue_destroy(queue* q);
float dequeue(queue *q);
bool enqueue(queue* q, float value);

float calculate_mean(queue* q);
float calculate_std_dev(queue* q, float mean);
float calculate_variance(queue* q, float mean);

// --------------------------------------------------------------

bool push(stack *mystack, int value);
int pop(stack *mystack);


#endif // QUEUE_H