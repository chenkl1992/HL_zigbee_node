#include "stdarg.h"
#include "printf-stdarg.h"
#include "log.h"
#include  <assert.h>

//LOG打印级别
volatile unsigned char log_level = LOG_LEVEL_ALL;

void log_level_set(uint8_t new_level)
{
  log_level = new_level;
}

/*******************************************************************************
*函数名: log_printf
*说明: 自定义LOG printf函数
*参数: format - 格式化字符串
*返回: 0 - OK
       -1 - ERROR
其他: 无
*******************************************************************************/
int log_printf(const char *format, ...)
{
    va_list args;
    int ret;
   
    va_start( args, format );
    
    ret = printk_va( 0, format, args );

    return ret;

}

