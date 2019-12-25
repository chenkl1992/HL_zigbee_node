/*******************************************************************************
* Filename : queue.c
* Version  : V1.0
********************************************************************************
* Note(s)  : 
*******************************************************************************/


/*******************************************************************************
*                                 INCLUDE FILES
*******************************************************************************/
#include "stdint.h"
#include "queue.h"

/*******************************************************************************
*                                 LOCAL DEFINES
*******************************************************************************/

/*******************************************************************************
*                             LOCAL GLOBAL VARIABLES
*******************************************************************************/

/*******************************************************************************
*                            LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
static uint8_t queue_is_full(queue_t* q);
static uint8_t queue_is_empty(queue_t* q);

/*******************************************************************************
*函数名: queue_init
*说明: 队列初始化
*参数: q - 队列ID
       p_data - 队列数据起始地址
       size_q - 队列大小 即队列元素数目
       size_e - 队列中单个元素大小
*返回: 无
*其他: 无
*******************************************************************************/
void queue_init(queue_t* q, void* p_data, uint32_t size_q, uint32_t size_e)
{
    if(q == (queue_t*)0)
      return;
    
    if(p_data == (void*)0)
      return;
  
	q->front = 0;
	q->rear = 0;
	q->p_data = p_data;
	q->size.queue = size_q;
	q->size.element = size_e;
}

/*******************************************************************************
*函数名: queue_push
*说明: 队列中压入数据
*参数: q - 队列ID
       e - 压入的元素数据
*返回: 0 - OK
      -1 - ERROR
*其他: 无
*******************************************************************************/
int32_t queue_push(queue_t* q, void* e)
{
	uint8_t* p_dst;
	uint8_t* p_src;
	uint32_t i;

    if(q == (queue_t*)0)
        return -1;
    
    if(e == (void*)0)
        return -1;
    
	if (queue_is_full(q))
        return -1;
	
	p_dst = (uint8_t*)q->p_data + q->size.element * q->rear;
	p_src = e;
	for (i = 0; i < q->size.element; i++, p_dst++, p_src++)
	{
		*p_dst = *(uint8_t*)p_src;
	}

	q->rear = (q->rear + 1) % q->size.queue;

	return 0;
}

/*******************************************************************************
*函数名: queue_pop
*说明: 从队列取出数据
*参数: q - 队列ID
       e - 取出的队列元素数据
*返回: 0 - OK
      -1 - ERROR
*其他: 无
*******************************************************************************/
int32_t queue_pop(queue_t* q, void* e)
{
	uint8_t* p_dst;
	uint8_t* p_src;
	uint32_t i;

    if(q == (queue_t*)0)
        return -1;
    
    if (e == (void *)0)
		return -1;
    
	if (queue_is_empty(q))
		return -1;
    
	p_src = (uint8_t*)q->p_data + q->size.element * q->front;
	p_dst = e;
	for (i = 0; i < q->size.element; i++, p_dst++, p_src++)
	{
		*p_dst = *(uint8_t*)p_src;
	}

	q->front = (q->front + 1) % q->size.queue;

	return 0;
}

/*******************************************************************************
*函数名: queue_is_full
*说明: 判断队列是否已满
*参数: q - 队列ID
*返回: 0 - 未满
       1 - 已满
*其他: 无
*******************************************************************************/
static uint8_t queue_is_full(queue_t* q)
{
    if(q == (queue_t*)0)
        return 1;
  
	return ((q->rear + 1) % q->size.queue == q->front);
}

/*******************************************************************************
*函数名: queue_is_empty
*说明: 判断队列是否为空
*参数: q - 队列ID
*返回: 0 - 队列不为空
       1 - 队列为空
*其他: 无
*******************************************************************************/
static uint8_t queue_is_empty(queue_t* q)
{
    if(q == (queue_t*)0)
        return 1;
  
	return (q->front == q->rear);
}