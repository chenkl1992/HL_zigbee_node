/*******************************************************************************
* Filename : service_Shell.c
* Version  : V1.0
********************************************************************************
* Note(s)  : 
*******************************************************************************/


/*******************************************************************************
*                                 INCLUDE FILES
*******************************************************************************/
#include "stm32l0xx_hal.h"
#include "service_Shell.h"
#include "shell_call.h"
#include "string.h"
#include "usart.h"
#include "event.h"
/*******************************************************************************
*                                 LOCAL DEFINES
*******************************************************************************/
#define SHELL_EVENT_NUM                                                   4


/*******************************************************************************
*                             LOCAL GLOBAL VARIABLES
*******************************************************************************/
static queue_t queue_Shell;
static event_t event_Shell;
static event_element_t element_Shell[SHELL_EVENT_NUM];
static struct {
    char data[128];
    uint32_t count;
}shell_uart = {0};

/*******************************************************************************
*                            LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

/*******************************************************************************
*函数名: shell_Rx_Enable
*说明: 使能接收中断
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void shell_Rx_Enable(void)
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/*******************************************************************************
*函数名: service_Shell_init
*说明: Shell服务初始化
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void service_Shell_init(void)
{
    event_create(event_Shell_id_get(), 
                 &queue_Shell, 
                 element_Shell, 
                 SHELL_EVENT_NUM);
    
    shell_printf(COM1SHELL_MSG_BANNER);
    shell_printf(COM1SHELL_MSG_PROMPT);
    shell_Rx_Enable();
}

/*******************************************************************************
*函数名: shell_line_get
*说明:   获取并回显命令
*参数:   串口数据
*返回:   0 - OK
         -1 - ERROR
其他:    无
*******************************************************************************/
int shell_line_get(uint32_t c)
{
  char esc[8];
  uint8_t esc_index = 0;
  switch (c)
  {
  case LF:
    shell_printf("%c", CR);
    shell_printf("%c", c);
    shell_uart.data[shell_uart.count] = '\0'; 
    shell_printf(COM1SHELL_MSG_PROMPT);
    return(shell_uart.count);
  case CR:
    shell_printf("%c", c);
    shell_printf("%c", LF);
    shell_uart.data[shell_uart.count] = '\0';
    shell_printf(COM1SHELL_MSG_PROMPT);
    return(shell_uart.count);
  case ABORT_CHAR:
    memset(&shell_uart, 0, sizeof(shell_uart));
    shell_printf(CRLF);
    shell_printf(COM1SHELL_MSG_PROMPT );
    break;
  case BKSPACE_CHAR:
    if(shell_uart.count > 0)
    {
      shell_printf("\b \b" );
      shell_uart.count--;
    }
    break;
    
  case ESC:
    memset(esc, 0, sizeof(esc));
    esc_index = 0;
    
    if(esc_index < sizeof(esc))
      esc[esc_index ++] = c;
    break;
    
  case MI_B:
    if(esc[esc_index - 1] == ESC)
    {
      if(esc_index < sizeof(esc))
        esc[esc_index ++] = c;
    }
    else
    {
      shell_printf("%c", c);
      if(shell_uart.count < sizeof(shell_uart.data) - 1)
        shell_uart.data[shell_uart.count++] = c;
    }
    break;
    
  default:
    if((esc[esc_index - 2] == ESC) && (esc[esc_index - 1] == MI_B))
    {
      if(esc_index < sizeof(esc))
        esc[esc_index ++] = c;
      esc_index = 0;
    }
    else
    {    
      shell_printf("%c", c);
      if(shell_uart.count < sizeof(shell_uart.data) - 1)
        shell_uart.data[shell_uart.count++] = c;
    }
    break;
  }
  return 0;
}

/*******************************************************************************
*函数名: shell_line_data_get
*说明:   获取命令行数据
*参数:   data - 获取到的命令行字符串
*返回:  命令行长度
*其他:  无
*******************************************************************************/
int32_t shell_line_data_get(char **data)
{
    *data = shell_uart.data;
    
    return shell_uart.count;
}

/*******************************************************************************
*函数名: service_Shell
*说明: Shell服务
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void service_Shell(void)
{
  int32_t ret;
  char *line;
  char *reply;
  event_element_t element;
  
  ret = event_wait(&event_Shell, &element, 0);
  if(ret == 0)
  {
    switch(element.type)
    {
    case EVT_SHELL_DATA:
      {
        //保存串口输入直到结束符
        if(shell_line_get((uint32_t)element.data)>0)
        {
          //获取所输入内容
          if(shell_line_data_get(&line)>0)
          {
            //处理数据
            Shell_exec((signed char *)line, 
                       (signed char **)&reply);
            memset(&shell_uart, 0, sizeof(shell_uart));
          }
        }
        shell_Rx_Enable();
      } 
      break;
    }
  }
}

/*******************************************************************************
*函数名: event_Shell_id_get
*说明: Shell服务id
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
event_t *event_Shell_id_get(void)
{
    return &event_Shell;
}

/*******************************************************************************
*函数名: USART2_IRQHandler
*说明:   shell 命令上数据接收中断
*参数:  无
*返回: 无
其他: 无
*******************************************************************************/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  uint8_t mdata = 0;
  event_element_t element;
  /* USER CODE END USART2_IRQn 0 */
  if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
  {
    mdata = (uint8_t)huart2.Instance->RDR;
    /* USER CODE BEGIN USART2_IRQn 1 */
    element.data = (void*)mdata;
    element.size = 1;
    element.type = EVT_SHELL_DATA;
    event_send(event_Shell_id_get(), &element);
  }
  /* USER CODE END USART2_IRQn 1 */
}
