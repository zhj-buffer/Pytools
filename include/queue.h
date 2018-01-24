#ifndef QUEUE_H
#define QUEUE_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
typedef struct Node
{
    int data;
    struct Node* next;
    struct Node* prev;
}queueNode;

typedef struct linkQueue
{
    unsigned int size;
    queueNode* head;
    queueNode* tail;
}Queue;

Queue* InitQueue(Queue *queue);
void EnQueue(Queue*, int);
int DeQueue(Queue* queue);
void PrintQueue(Queue* queue);
int IsNull(Queue* queue);
void DelQueue(Queue* queue);

#ifdef __cplusplus
};
#endif

#endif
