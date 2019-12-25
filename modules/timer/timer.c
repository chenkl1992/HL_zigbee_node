/*******************************************************************************
* Filename : timer.c
* Version  : V1.0
********************************************************************************
* Note(s)  : 
*******************************************************************************/


/*******************************************************************************
*                                 INCLUDE FILES
*******************************************************************************/
#include "stdint.h"
#include "timer.h"
#include <time.h>
#include "rtc.h"
#include "stm32l0xx_hal.h"
/*******************************************************************************
*                                 LOCAL DEFINES
*******************************************************************************/


/*******************************************************************************
*                             LOCAL GLOBAL VARIABLES
*******************************************************************************/
timer_t *timer_list[TIMER_NUM] = {0};


/*******************************************************************************
*                            LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
static int32_t timer_list_insert(timer_t *p_timer);

static uint32_t timer_tick_get(void);

/*******************************************************************************
*函数名: timer_create
*说明: Timer 创建
*参数: p_timer - 定时器 ID
       period - 定时器周期
       opt - 定时器操作选项
       callback - 定时器回调函数
*返回: 0 - OK
       -1 - ERROR
*其他: 无
*******************************************************************************/
int32_t timer_create(timer_t *p_timer,
                     uint32_t period,
                     uint32_t opt,
                     timer_callback callback)
{
    int32_t ret;
  
    if(p_timer == (timer_t *)0)
        return -1;
  
    if(callback == (timer_callback)0)
        return -1;
    
    p_timer->callback = callback;
    p_timer->opt = opt;
    p_timer->tick.period = period;
    p_timer->state = TIMER_STATE_STOP;
    
    ret = timer_list_insert(p_timer);
    
    return ret;
}

/*******************************************************************************
*函数名: timer_start
*说明: 定时器启动
*参数: p_timer - 定时器 ID
*返回: 0 - OK
       -1 - ERROR
*其他: 无
*******************************************************************************/
void timer_start(timer_t *p_timer)
{
    if(p_timer == (timer_t *)0)
        return;
  
    p_timer->tick.start = timer_tick_get();
    
    p_timer->state = TIMER_STATE_START;
}

/*******************************************************************************
*函数名: timer_stop
*说明: 定时器停止
*参数: p_timer - 定时器 ID
*返回: 无
*其他: 无
*******************************************************************************/
void timer_stop(timer_t *p_timer)
{
    if(p_timer == (timer_t *)0)
        return;
    
    p_timer->state = TIMER_STATE_STOP;
}

/*******************************************************************************
*函数名: timer_list_null_find
*说明: 查找定时器中为NULL的位置
*参数: p_timer - 输出为NULL的定时器 ID地址
*返回: 0 - OK
       -1 - ERROR
*其他: 无
*******************************************************************************/
static int32_t timer_list_null_find(timer_t ***p_timer)
{
    timer_t **p;
    uint32_t i;
    
    if(p_timer == (timer_t ***)0)
        return -1;
    
    p = &timer_list[0];
    
    for(i = 0; i < TIMER_NUM; i++, p++)
    {
        if(*p == (timer_t *)0)
        {
            *p_timer = p;
            
            return 0;
        }
    }

    return -1;
}

/*******************************************************************************
*函数名: timer_list_insert
*说明: 定时器缓存队列中插入定时器ID
*参数: p_timer - 定时器 ID
*返回: 0 - OK
       -1 - ERROR
*其他: 无
*******************************************************************************/
static int32_t timer_list_insert(timer_t *p_timer)
{
    timer_t **p;
    int32_t ret;
    
    if(p_timer == (timer_t*)0)
        return -1;
    
    ret = timer_list_null_find(&p);
    if(ret != 0)
    {
        return -1;
    }
        
    *p = p_timer;
    
    return 0;
}

/*******************************************************************************
*函数名: timer_sched
*说明: 定时器定时处理 在systick中断函数中调用
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void timer_sched(void)
{
    uint32_t tick_now;
    timer_t **p;

    tick_now = timer_tick_get();
    for(p = &timer_list[0]; *p != (timer_t *)0; p++)
    {
        if((*p)->state == TIMER_STATE_STOP)
            continue;

        if(tick_now - (*p)->tick.start >= (*p)->tick.period)
        {
            if(((*p)->opt & TIMER_OPT_MODE_ONCE) == TIMER_OPT_MODE_ONCE)
            {
                (*p)->state = TIMER_STATE_STOP;
                (*p)->tick.start = 0;
            }
            else
            {
                (*p)->tick.start = tick_now;
            }
            
            if((*p)->callback != (timer_callback)0)
            {
                (*p)->callback();
            }
        }
    }
}

/*******************************************************************************
*函数名: timer_tick_get
*说明: 定时器tick获取
*参数: 无
*返回: tick值 即ms值
*其他: 无
*******************************************************************************/
static uint32_t timer_tick_get(void)
{
    uint32_t tick;
    
    //extern uint32_t bsp_sys_tick_get(void);
    
    tick = HAL_GetTick();
    
    return tick;
}

/*******************************************************************************
*函数名: 
*说明: BCD时间转时间戳
*参数: 无
*返回: 时间戳
*其他: 无
*******************************************************************************/
uint32_t Time_To_Unix()
{ 
  struct tm stmT;
  uint32_t time = 0;
  RTC_DateTypeDef sdate;
  RTC_TimeTypeDef stime;
  
  __disable_irq();
  HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sdate, RTC_FORMAT_BIN);
   
  stmT.tm_year = sdate.Year;
  stmT.tm_mon = sdate.Month;  
  stmT.tm_mday = sdate.Date;  
  stmT.tm_hour = stime.Hours;  
  stmT.tm_min = stime.Minutes;  
  stmT.tm_sec = stime.Seconds;
  
  time = mktime(&stmT);
  __enable_irq();
  return time;
}



