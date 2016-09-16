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

#define QUEUE_SIZE      256
#define ADC_QUEUE_SIZE  256

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

typedef struct adc_queue
{
    uint16 front;
    uint16 rear;
    uint16 count;
    uint16 container[ADC_QUEUE_SIZE];
}adc_queue;

extern void enqueue(queue* q, ipc_message message);
extern ipc_message* dequeue(queue* q);
extern ipc_message* peek(queue* q);
extern bool is_empty(queue* q);

// ADC queue functions, basically just copy-paste with different types
// would like to know if there were any ways to make container for diff
// function types like in C++, ie. foo<T>
extern void adc_enqueue(adc_queue* q, uint16 message);
extern uint16* adc_dequeue(adc_queue* q);
extern uint16* adc_peek(adc_queue* q);
extern bool adc_is_empty(adc_queue* q);


/* [] END OF FILE */
