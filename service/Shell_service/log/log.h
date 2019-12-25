#ifndef __LOG_H
#define __LOG_H

#define DEBUG_PRINTF_EN                                                 1

#include <stdarg.h>
#include "stm32l0xx_hal.h"
int log_printf(const char *format, ...);

void log_level_set(uint8_t new_level);

extern volatile unsigned char log_level;		/* 设置成变量而不是宏，便于在串口修改调试等级，打印不同的日志信息 */

#define  LOG_LEVEL_OFF            0x00   // 全部关闭
#define  LOG_LEVEL_ERR            0x01   // 一般错误(Error)
#define  LOG_LEVEL_WAR            0x02   // 警告(可能引起错误)(Warn)
#define  LOG_LEVEL_INF            0x04   // 运行过程消息(Info)
#define  LOG_LEVEL_DBG            0x08   // 调试信息(Debug)
#define  LOG_LEVEL_CMD            0x10   // 调试信息(Debug)
#define  LOG_LEVEL_ALL            0xFF   // 全部开启

#define  LOG_TRACE			      log_printf


#define logdbg(fmt, ...)  \
			do{\
				if(log_level & LOG_LEVEL_DBG)\
				{\
					LOG_TRACE("[DBG]time:[%010d]%s-%d: " fmt "", HAL_GetTick(), __func__,__LINE__, ##__VA_ARGS__);\
				}\
			}while(0)

#define loginf(fmt, ...)  \
			do{\
				if(log_level & LOG_LEVEL_INF)\
				{\
					LOG_TRACE("[INF]time:[%010d]%s-%d: " fmt "" , HAL_GetTick(), __func__,__LINE__, ##__VA_ARGS__);\
				}\
			}while(0);
			
#define logwar(fmt, ...)  \
			do{\
				if(log_level & LOG_LEVEL_WAR)\
				{\
					LOG_TRACE("[WAR]time:[%010d]%s-%d: " fmt "" , HAL_GetTick(), __func__,__LINE__, ##__VA_ARGS__);\
				}\
			}while(0)
	
#define logerr(fmt, ...)  \
			do{\
				if(log_level & LOG_LEVEL_ERR)\
				{\
					LOG_TRACE("[ERR]time:[%010d]%s-%d: " fmt "" , HAL_GetTick(), __func__,__LINE__, ##__VA_ARGS__);\
				}\
			}while(0)
                          
#define logcmd(fmt, ...)  \
			do{\
				if(log_level & LOG_LEVEL_CMD)\
				{\
					LOG_TRACE(""fmt"" , ##__VA_ARGS__);\
				}\
			}while(0)                          

#endif
