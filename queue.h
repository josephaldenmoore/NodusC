/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <stdbool.h>
#include <project.h>

#define QUEUE_SIZE  256

typedef struct ipc_message
{
    // header
    uint8 address;
    // body
    char* data;
} ipc_message;

typedef struct queue
{
    uint16 front;
    uint16 rear;
    uint16 count;
    ipc_message container[QUEUE_SIZE];
}queue;

extern queue create_queue();
extern void enqueue(queue* q, ipc_message message);
extern ipc_message* dequeue(queue* q);
extern ipc_message* peek(queue* q);
extern bool is_empty(queue* q);


/* [] END OF FILE */
