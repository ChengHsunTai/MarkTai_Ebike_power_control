#include "queue.h"
#include <stdlib.h>
#include <stdio.h>

void queue_init(Queue *q) {
    q->head = NULL;
    q->tail = NULL;
    q->size = 0;
}

bool queue_is_empty(Queue *q) {
    return (q->size == 0);
}

bool queue_enqueue(Queue *q, float value) {
    QueueNode *node = (QueueNode *)malloc(sizeof(QueueNode));
    if (!node) return false;

    node->data = value;
    node->next = NULL;

    if (q->tail == NULL) {
        q->head = node;
        q->tail = node;
    } else {
        q->tail->next = node;
        q->tail = node;
    }
    q->size++;
    return true;
}

bool queue_dequeue(Queue *q, float *value) {
    if (queue_is_empty(q)) return false;

    QueueNode *node = q->head;
    *value = node->data;

    q->head = node->next;
    if (q->head == NULL) {
        q->tail = NULL;
    }
    free(node);
    q->size--;
    return true;
}

void queue_clear(Queue *q) {
    float tmp;
    while (queue_dequeue(q, &tmp)) {
        // 清到底
    }
}
