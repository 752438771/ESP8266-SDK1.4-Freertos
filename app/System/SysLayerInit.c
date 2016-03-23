
#include "user_config.h"
#ifndef MakeUSER2
#include "SysLayerInit.h"
//#include "os_type.h"
//#include "osapi.h"


extern bool IsConnect;
//os_event_t    LesTaskQueue[LesTaskQueueLen];
//内部变量
static uint16_t  mDelay;
//内部函数

//os_timer_t SystemSysTick;

#define IO_SET_OUTPUT(VALUE)    \
{   \
	if(VALUE) gpio_output_set(BIT0, 0, BIT0, 0);\
	else      gpio_output_set(0, BIT0, BIT0, 0);\
}

/*******************************************************************************
* 名称: SysTick_Handler
* 功能: 系统时钟节拍1MS
* 形参:
* 返回: 无
* 说明:
*******************************************************************************/
void //ICACHE_FLASH_ATTR
SysTickStart(void)
{
	//os_timer_disarm(&SystemSysTick);
	//os_timer_setfn(&SystemSysTick,(os_timer_func_t*)SysTick_Handler,NULL);
	//os_timer_arm(&SystemSysTick, 1, 1);
}

void //ICACHE_FLASH_ATTR
SysTick_Handler(void)
{
  static u16 Tick_1S    =0;

  mSysSec++;
  mSysTick++;
  if(mDelay)mDelay--;

  if(++Tick_1S >= 1000)
  {
    Tick_1S = 0;
	system_soft_wdt_feed();
  }
}



/*******************************************************************************
* 名称: Sys_GetMac
* 功能: mac计算
* 形参:       
* 返回: 无
* 说明: 
*******************************************************************************/
void //ICACHE_FLASH_ATTR
 Sys_GetMac(u8 *mac)
{ 
  u32 Device_Serial0, Device_Serial1, Device_Serial2;  
  u8  i,Dev1[4],Dev2[4],Dev3[4];
  
  Device_Serial0 = *(uint32_t*)(0x1FFFF7AC);
  Device_Serial1 = *(uint32_t*)(0x1FFFF7B0);
  Device_Serial2 = *(uint32_t*)(0x1FFFF7B4);
  mac[0]=(u8 )((Device_Serial0>>1)+(Device_Serial0>>16));
  mac[1]=(u8 )((Device_Serial1>>2)+(Device_Serial1>>16));
  mac[2]=(u8 )((Device_Serial2>>3)+(Device_Serial2>>16));    	   /*MAC最后3字节利用STM的唯一ID计算，特别注意此方法可能存在重复，需要配置应用软件做出厂检测MAC的唯一性*/
  
  for(i=0;i<4;i++)Dev1[i] = *(uint8_t*)(0x1FFFF7AC+i);
  for(i=0;i<4;i++)Dev2[i] = *(uint8_t*)(0x1FFFF7B0+i);
  for(i=0;i<4;i++)Dev3[i] = *(uint8_t*)(0x1FFFF7B4+i);
  mac[3]=Dev1[0]+Dev2[1]+Dev3[2]+Dev2[3];
  mac[4]=Dev2[0]+Dev3[1]+Dev1[2]+Dev3[3];
  mac[5]=Dev3[0]+Dev1[1]+Dev2[2]+Dev1[3];
  
  if(mac[0]&0x01)mac[0]++;
}

/*******************************************************************************
* 名称: Sys_LayerInit
* 功能: 系统初始化
* 形参:       
* 返回: 无
* 说明: 
*******************************************************************************/
void //ICACHE_FLASH_ATTR
 Sys_LayerInit(void)
{

	//InterruptIOInit();
  SysTask.m_Hs6200Ok = 100;
  SysTask.m_LedCnt = 5;
  SysTask.m_WifiState = WIFI_SMARTLINK;
  SysTask.mNetwork.mUpdate = 0;
  mSysSec  = 0;
  mSysTick = 0;
//	IO_SET_OUTPUT(0);
  WifiInit();
}




/*******************************************************************************
* 名称:LED_FLASH
* 功能:Les系统计时
* 形参:
* 返回: 无
* 说明:
******************************************************************************/
void //ICACHE_FLASH_ATTR
LED_FLASH(void)
{
	static u8 value=0;
//				os_printf("LED_FLASH= %d= %d\r\n",SysTask.m_LedCnt,value);
	if(SysTask.m_LedCnt == 1)
	{
		switch(SysTask.m_WifiState)
		{
		case WIFI_SMARTLINK:
			SysTask.m_LedCnt = 2;
			break;
		case WIFI_WLAN:
			SysTask.m_LedCnt = 4;
			break;
		case WIFI_TCP_FAIL:
			SysTask.m_LedCnt = 6;
			break;
		case WIFI_TCP_SUCCESS:
			SysTask.m_LedCnt = 8;
			break;
		default:
			//os_printf("LED_FLASH\r\n");
			SysTask.m_LedCnt = 0;
			value = 1;
			break;
		}
		value = ~value;
	}
	else if(SysTask.m_LedCnt > 0) SysTask.m_LedCnt--;
}

/*******************************************************************************
* 名称:SysTask_Tick
* 功能:Les系统计时
* 形参:
* 返回: 无
* 说明:
******************************************************************************/
void //ICACHE_FLASH_ATTR
 SysTask_Tick(void)
{
  if(mSysTick == 0)return;//系统节拍计时器
  mSysTick=0;

  if(SysTask.m_Hs6200Ok > 200){SysTask.m_Hs6200Ok = 0;SysTask.m_CloudOk = 6;}
  else if(SysTask.m_Hs6200Ok > 99)SysTask.m_Hs6200Ok++;

//  if(SysTask.m_CloudOk > 200)SysTask.m_CloudOk = 0;
//  else if(SysTask.m_CloudOk > 5)SysTask.m_CloudOk++;

//  LetsNetWork_Tick();                                   //协议节拍
  if(mSysSec >= 1000){                                //秒计数
    LED_FLASH();                                          //LED节拍
    mSysSec = 0;                                        //秒计数清零
//    if(SysTask.m_WifiState > WIFI_SMARTLINK)
//    	UDPs_Tick();                                        //UDP任务

#ifdef CloudKey
    if(IsConnect)Cloud_Tick();                                       //云任务节拍
    if(mCloudState.Ackwait > 0)mCloudState.Ackwait--;   //服务器应答等待时间
#endif

//    if(SysTask.eTime)SysTick_DateAndTime();             //定时/时钟任务

    if(SysTask.nCtCnt > 0)
    {
    	SysTask.nCtCnt--;             //允许新灯加入时间
    	//os_printf("允许加入时间 is %d\r\n",SysTask.nCtCnt);
    }

//    PowerOffToSmartLink();
//    LinkTcpReConnect();
  }


}

/*******************************************************************************
* 名称:LesInit
* 功能:Les系统
* 形参:
* 返回: 无
* 说明:
******************************************************************************/
void //ICACHE_FLASH_ATTR
LesTask(void)
{

#ifdef CloudKey
	if(!SysTask.m_CloudOk && IsConnect)
		Cloud_Client_Send();        //发送数据-》云端
#endif

//	Le_CheckStationState();
    SysTask_Tick();

	//system_os_post(LesTaskPrio, 0, 0);
}
/*******************************************************************************
* 名称:LesInit
* 功能:Les系统
* 形参:
* 返回: 无
* 说明:
******************************************************************************/
void //ICACHE_FLASH_ATTR
LesInit(void)
{
	//system_os_task(LesTask, LesTaskPrio, LesTaskQueue, LesTaskQueueLen);
	//system_os_post(LesTaskPrio, 0, 0);
}
/*******************************************************************************
* 名称:
* 功能:
* 形参:
* 返回: 无
* 说明:
*******************************************************************************/
#pragma optimize=none
void //ICACHE_FLASH_ATTR
DelayUs(uint16_t nCount)
{
	for(;nCount>0;nCount--)
		os_delay_us(1);
}
#pragma optimize=none
void //ICACHE_FLASH_ATTR
 DelayMs(uint16_t nCount)
{
	for(;nCount>0;nCount--)
	 os_delay_us(1000);
}

/*******************************************************************************
* 名称: Strcmp()
* 功能:
* 形参:
* 返回: 无
* 说明:
******************************************************************************/
bool //ICACHE_FLASH_ATTR
 Strcmp(u8* str1,u8* str2,u8 len)
{
  for(;len>0;len--){
    if(*str1++ != *str2++)return FALSE;
  }
  return TRUE;
}

/*******************************************************************************
* 名称: GetCRC
* 功能:
* 形参:
* 返回: 无
* 说明:
*******************************************************************************/
u8 //ICACHE_FLASH_ATTR
    GetCRC(u8 *buf,u8 len)
{
  u8 crc = 0;
  for(;len>0;len--){
    crc += *buf++;
  }
  return(~crc);
}
#endif
