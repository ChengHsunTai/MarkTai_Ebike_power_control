/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    queue.h
  * @brief   Simple linked-list queue interface
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __QUEUE_H__
#define __QUEUE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* USER CODE BEGIN Includes */
/* 如果你真的需要 main.h 裡的東西，再加這行 */
// #include "main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Exported types */
typedef struct QueueNode {
    float data;                 // 之後你可以改成你要的型別
    struct QueueNode *next;
} QueueNode;

typedef struct {
    QueueNode *head;
    QueueNode *tail;
    size_t     size;
} Queue;
/* USER CODE END Exported types */

/* USER CODE BEGIN Prototypes */
void queue_init(Queue *q);
bool queue_is_empty(Queue *q);
bool queue_enqueue(Queue *q, float value);
bool queue_dequeue(Queue *q, float *value);
void queue_clear(Queue *q);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __QUEUE_H__ */
