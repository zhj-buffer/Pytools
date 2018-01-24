#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include "queue.h"

void DelQueue(Queue* queue)
{
    while(queue->head->next != queue->tail)
    {
        DeQueue(queue);
    }

    free(queue->tail);
    free(queue->head);
}

void PrintQueue(Queue* queue)
{
    if(IsNull(queue))
    {
        printf("empty queue.\n");
        return ;
    }
    queueNode* curNode= queue->head->next;
    while(curNode != queue->tail)
    {
        if(curNode->next != queue->tail)
        {
            printf("%d==>", (curNode->data));
        }else
        {
            printf("%d ", (curNode->data));

        }
        curNode = curNode->next;
    }
    printf("\n");

}

Queue* InitQueue(Queue *queue)
{
    queue->tail = (queueNode*)malloc(sizeof(queueNode));//warning
    queue->head = (queueNode*)malloc(sizeof(queueNode));//warning
    queue->tail->data = -1;
    queue->head->data = -1;
    queue->head->next = queue->tail;
    queue->tail->prev = queue->head;
    queue->size = 0;

    return queue;
}
//入队列
void EnQueue(Queue* queue, int data)
{
    queueNode * newNode = (queueNode*)malloc(sizeof(queueNode));//warning
    newNode->data = data;
    newNode->next = NULL;
    if (queue->head->next == queue->tail) {
        queue->head->next = newNode;
        newNode->prev = queue->head;
        queue->tail->prev = newNode;
        newNode->next = queue->tail;
    } else {
        newNode->next = queue->head->next;
        queue->head->next->prev = newNode;
        queue->head->next = newNode;
        newNode->prev = queue->head;
    }

    queue->size++;
}

int DeQueue(Queue* queue)
{
    if (queue->tail->prev != queue->head) {
        queueNode *popNode = queue->tail->prev;
        int popValue = popNode->data;
        queue->tail->prev = popNode->prev;
        popNode->prev->next = queue->tail;
        free(popNode);//warning
        queue->size--;
        return popValue;
    } else {
        return -1;
    }
}

//1 means Null
int IsNull(Queue* queue)
{
    return (queue->head->next == queue->tail);
}

#ifdef __cplusplus
};
#endif