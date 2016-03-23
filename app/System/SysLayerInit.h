#ifndef __SYSLAYERINIT_H__
#define __SYSLAYERINIT_H__
#include "user_config.h"
#ifndef MakeUSER2
#include"c_types.h"
//#include "osapi.h"


#ifdef _MAININC_
#define EXTERN
#else
#define EXTERN extern
#endif


#define LesTaskPrio        1
#define LesTaskQueueLen    10
#define           NET_SYSNAME          16
/* 头文件  ------------------------------------------------------------*/


#define         BOOL            bool
#define         VOID            void    


//联合体由于数据大小端问题--所有变量必须使用字节声明
//设备信息
#pragma pack(1)
typedef enum
{
  WIFI_SMARTLINK=0,
  WIFI_TCP_FAIL,
  WIFI_TCP_SUCCESS,
  WIFI_WLAN,
}WifiState;

typedef struct
{
  unsigned char   mDhcp:1,            //DHCP
  	  	  	  	  	mUpdate:2,             //mUpdate
                    mLogin:1,           //服务器登录
                    mRemark1:1,         //
                    mRemark2:1,         //
                    mRemark3:1,
                    mRemark4:1;
}NETWORK_STATE;
typedef struct
{
	NETWORK_STATE  mNetwork;

  u8              mUpPackALL;                  //总包数
  bool           mUpApp;
  bool           mUpCloud;
  u8             mLogin;
  
  u8              nName[NET_SYSNAME];           //网关名字

 uint16_t            nTick;                        //节拍器
#define           NET_RESAVE_TIME      3       //3S
 volatile u16             nSaveTick;                    //保存倒计时
#define           NET_CONNET_OPEN_TIME 600      //600s
  u8              nSentCnt;                     //重发包次数
  u16             nCtCnt;                       //连接允许计数器
  
  u8              nMode;                        //Net状态
  u8              nSBMode;                      //Net子状态
  u8              nTxAck;                       //Net上一次是否应答
  u16             mTxDelay;                     //发送等待
  u16             nTxWait;                      //等待计数
  u8              mRFreset;
  
  BOOL            eTime;
  u8              mTimeSec;
  u8              mTimeMin;
  u8              mTimeHour;

  u8              m_Hs6200Ok;
  u8              m_CloudOk;

  u8              m_CloseCnt;                   //smartlink计数
  u16             m_LedCnt;                     //Led计数
  u8             m_WificonnectTime;                     //Led计数
  u8             m_LogintTime;                     //Led计数
  WifiState       m_WifiState;
}SYS_TASK;
#pragma pack()




/* 全局变量 -----------------------------------------------------------*/
 uint8_t      mSysIWDGDog;            //软狗标记
 uint32_t     mSysSoftDog;            //软狗计数器
 uint16_t     mSysTick;               //节拍器
 uint16_t     mSysSec;                //节拍器
 SYS_TASK SysTask;


EXTERN void SysTickStart(void);
void SysTick_Handler(void);
//EXTERN void  Sys_DelayMS(uint16_t nms);
EXTERN void  Sys_GetMac(u8 *mac);
EXTERN void  Sys_LayerInit(void);
EXTERN void  Sys_IWDGConfig(u16 time);
EXTERN void  Sys_IWDGReloadCounter(void);
EXTERN void  Sys_1s_Tick(void);

EXTERN void  DelayUs(uint16_t nCount);
EXTERN void  DelayMs(uint16_t nCount);
EXTERN void  Strcpy(u8* str1,u8* str2,u8 len);
EXTERN bool  Strcmp(u8* str1,u8* str2,u8 len);
u8 GetCRC(u8 *buf,u8 len);
//void Sprintf_a_to_b(char* buf,char* temp,u8 len);
#endif
#endif
