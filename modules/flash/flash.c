/*******************************************************************************
* Filename : flash.c
* Version  : V1.0
********************************************************************************
* Note(s)  : 
*******************************************************************************/


/*******************************************************************************
*                                 INCLUDE FILES
*******************************************************************************/
#include "flash.h"
#include "stm32l0xx_hal.h"

/*******************************************************************************
*                                 LOCAL DEFINES
*******************************************************************************/

/*******************************************************************************
*                             LOCAL GLOBAL VARIABLES
*******************************************************************************/

/*******************************************************************************
*                            LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

/*******************************************************************************
** 功能	      FLASH初始化
** 参数       无
** 返回值	    无
** 注意       
** 修改日志
*******************************************************************************/
void FLASH_Init(void)
{

}

/*******************************************************************************
** 功能	      EEPROM初始化
** 参数       无
** 返回值	    无
** 注意       此函数为兼容其他EEPROM而写，为空函数
** 修改日志
*******************************************************************************/
void EEPROM_Init(void)
{
    
}

/*******************************************************************************
** 功能	      向EEPROM写一字节数据
** 参数       add:存储器地址,从0开始
              data:要写入的数据字节
** 返回值	    无
** 注意       
** 修改日志
*******************************************************************************/
void EEPROM_WriteOneByte(uint32_t add, uint8_t data)
{
    HAL_FLASHEx_DATAEEPROM_Unlock();//解锁EEPROM
    add += EEPROM_BASE_ADDR;//计算EEPROM实际地址
    if(add > EEPROM_END_ADDR)//地址超出范围
    {
        HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM上锁 
        return;
    }
    if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, add, data) == HAL_OK)//向EEPROM写一字节数据
    {
        
    }
    HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM上锁 
}

/*******************************************************************************
** 功能	      从EEPROM读一字节数据
** 参数       add:存储器地址,从0开始
** 返回值	    读出的数据
** 注意       
** 修改日志
*******************************************************************************/
uint8_t EEPROM_ReadOneByte(uint32_t add)
{
    return *(__IO uint8_t*)(EEPROM_BASE_ADDR+add);
}

/*******************************************************************************
** 功能	      向EEPROM写一页数据
** 参数       add:存储器地址,从0开始
              p_wbuf:要写入的数据缓存指针
              nbyte：要写入的数据量，单位：字节
** 返回值	    无
** 注意       此函数为兼容其他EEPRO而写，无页地址对齐和页大小限制
** 修改日志
*******************************************************************************/
void EEPROM_WritePage(uint32_t add, uint8_t * p_wbuf, uint32_t nbyte)
{
    HAL_FLASHEx_DATAEEPROM_Unlock();//解锁EEPROM
    add += EEPROM_BASE_ADDR;//计算EEPROM实际地址
    /*  编程EEPROM */
    while(nbyte>0)
    {
        if(add > EEPROM_END_ADDR)//地址超出范围
        {
            HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM上锁  
            return;
        }
        if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, add, *p_wbuf) == HAL_OK)//向EEPROM写一字节数据
        {
            
        }  
        add++;
        p_wbuf++;
        nbyte--;
    }
    HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM上锁 
}

/*******************************************************************************
** 功能	      向EEPROM写多个数据
** 参数       add:存储器地址,从0开始
              p_wbuf:要写入的数据缓存指针
              nbyte：要写入的数据量，单位：字节
** 返回值	    无
** 注意       
** 修改日志
*******************************************************************************/
void EEPROM_WriteBytes(uint32_t add, uint8_t * p_wbuf, uint32_t nbyte)
{
    HAL_FLASHEx_DATAEEPROM_Unlock();//解锁EEPROM
    add += EEPROM_BASE_ADDR;//计算EEPROM实际地址
    /*  编程EEPROM */
    while(nbyte>0)
    {
        if(add > EEPROM_END_ADDR)//地址超出范围
        {
            HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM上锁  
            return;
        }
        if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, add, *p_wbuf) == HAL_OK)//向EEPROM写一字节数据
        {
            
        }  
        add++;
        p_wbuf++;
        nbyte--;
    }
    HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM上锁 
}

/*******************************************************************************
** 功能	      从EEPROM读取多个数据
** 参数       add:存储器地址,从0开始
              p_rbuf:读取到的数据存放的缓存指针
              nbyte：要读取的数据量，单位：字节
** 返回值	    无
** 注意       
** 修改日志
*******************************************************************************/
void EEPROM_ReadBytes(uint32_t add, uint8_t * p_rbuf, uint32_t nbyte)
{
    add += EEPROM_BASE_ADDR;//计算EEPROM实际地址    
    while(nbyte>0)
    {
        *p_rbuf = *(__IO uint8_t*)add;
        add++;
        p_rbuf++;
        nbyte--;
    }
}
