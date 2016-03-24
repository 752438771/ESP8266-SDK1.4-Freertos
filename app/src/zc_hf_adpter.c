/**
******************************************************************************
* @file     zc_hf_adpter.c
* @authors  cxy
* @version  V1.0.0
* @date     10-Sep-2014
* @brief    Event
******************************************************************************
*/
#include <zc_protocol_controller.h>
#include <zc_timer.h>
#include <zc_module_interface.h>
#include <zc_hf_adpter.h>
#include <stdlib.h>
#include <Ac_cfg.h>
#include <ac_api.h>
#include <bmd.h>
#include "espconn.h"
#include "flash_api.h"
#include "ets_sys.h"
#include "user_config.h"
#include "spi_flash.h"
#include "upgrade.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "smartlink.h"
#include "sockets.h"


#define  FLASH_ADDRESS            0x200000

u8  g_u8EqVersion[]={0,0,0,0};      
u8  g_u8ModuleKey[ZC_MODULE_KEY_LEN] =DEFAULT_IOT_PRIVATE_KEY;
u64  g_u64Domain;
u8  g_u8DeviceId[ZC_HS_DEVICE_ID_LEN];

//u64 SUB_DOMAIN_ID	=	0xFFFFFFFFFFFFFFFF;//子域id

extern PTC_ProtocolCon  g_struProtocolController;
PTC_ModuleAdapter g_struHfAdapter;

MSG_Buffer g_struRecvBuffer;
MSG_Buffer g_struRetxBuffer;
MSG_Buffer g_struClientBuffer;

extern UARTStruct UART0Port;
extern PTC_OtaBuf g_struOtaBuf;

MSG_Queue  g_struRecvQueue;
MSG_Buffer g_struSendBuffer[MSG_BUFFER_SEND_MAX_NUM];
MSG_Queue  g_struSendQueue;

u8 g_u8MsgBuildBuffer[MSG_BULID_BUFFER_MAXLEN];
u8 g_u8ClientSendLen = 0;

u8 UART0RxBuf[UART0RX_RING_LEN];
u8 pCmdWifiBuf[UART0RX_RING_LEN];

u8 g_u8recvbuffer[HF_MAX_SOCKET_LEN];
ZC_UartBuffer g_struUartBuffer;
ESP_TimerInfo g_struEspTimer[ZC_TIMER_MAX_NUM];

u8  g_u8BcSendBuffer[100];
u32 g_u32BcSleepCount = 800;//800

struct sockaddr_in struRemoteAddr;

extern volatile unsigned long  g_ulStatus;
extern void AC_UartProcess(u8* inBuf, u32 datalen);
extern void uart0_tx_buffer(uint8 *buf, uint16 len);


/*************************************************
* Function: ESP_UartSend
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_UartSend(u8* inBuf, u32 datalen)
{
    uart0_tx_buffer(inBuf, (u16)datalen);
}
/*************************************************
* Function: ESP_ReadDataFromFlash
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_ReadDataFromFlash(u8 *pu8Data, u16 u16Len)
{
    if (SPI_FLASH_RESULT_OK != spi_flash_read(FLASH_ADDRESS, (u32 *)pu8Data, (u32)u16Len))
    {
        ZC_Printf("ESP_ReadDataFromFlash error\n");
    }
}
/*************************************************
* Function: ESP_WriteDataToFlash
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_WriteDataToFlash(u8 *pu8Data, u16 u16Len)
{
    if (SPI_FLASH_RESULT_OK != spi_flash_erase_sector(512))
    {
        ZC_Printf("spi_flash_erase_sector fail\n");
        return;
    }
    if(SPI_FLASH_RESULT_OK != spi_flash_write(FLASH_ADDRESS, (uint32 *)pu8Data, u16Len))
    {   
        ZC_Printf("spi_flash_write sector fail\n");
        return;
    }
}

/*************************************************
* Function: ESP_timer_callback
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_timer_callback(void *timer_arg)
{
    u8 u8TimeId = *(u8 *)timer_arg;
    TIMER_TimeoutAction(u8TimeId);
    TIMER_StopTimer(u8TimeId);
}
/*************************************************
* Function: ESP_StopTimer
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_StopTimer(u8 u8TimerIndex)
{
    os_timer_disarm(&g_struEspTimer[u8TimerIndex].timer);
}
/*************************************************
* Function: ESP_SetTimer
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 //ICACHE_FLASH_ATTR
ESP_SetTimer(u8 u8Type, u32 u32Interval, u8 *pu8TimeIndex)
{
    u8 u8TimerIndex;
    u32 u32Retval;
    os_timer_t* timer;
    u32Retval = TIMER_FindIdleTimer(&u8TimerIndex);
    if (ZC_RET_OK == u32Retval)
    {
        //ZC_Printf("Set timer: type is %u, index is %u, interval is %u\n", u8Type, u8TimerIndex, u32Interval);
        TIMER_AllocateTimer(u8Type, u8TimerIndex, (u8*)&g_struEspTimer[u8TimerIndex]);
        *pu8TimeIndex = u8TimerIndex;
        g_struEspTimer[u8TimerIndex].u8Index = u8TimerIndex;
        timer = &g_struEspTimer[u8TimerIndex].timer;
    	os_timer_disarm(timer);
        os_timer_setfn(timer, (os_timer_func_t *)ESP_timer_callback, &g_struEspTimer[u8TimerIndex].u8Index);//g_struEspTimer[u8TimerIndex].u8Index);
        os_timer_arm(timer, u32Interval, 0);
    }
    else
    {
        ZC_Printf("no idle timer\n");
    }
    
    return u32Retval;
}
/*************************************************
* Function: ESP_FirmwareUpdateFinish
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 //ICACHE_FLASH_ATTR
ESP_FirmwareUpdateFinish(u32 u32TotalLen)
{
    u32 ret;
    if (0 != g_struOtaBuf.u16DateUsed)
    {
        ZC_Printf("OTA Finished: used is %d\n", g_struOtaBuf.u16DateUsed);
        ret = ESP_FlashEraseAddWrite();
        if (ZC_RET_OK != ret)
        {
            ZC_Printf("ESP_FlashEraseAddWrite error3\n");
            return ret;
        }
    }
    system_upgrade_flag_set(UPGRADE_FLAG_FINISH);
    system_upgrade_reboot();
    return ZC_RET_OK;
}
/*************************************************
* Function: ESP_FirmwareUpdate
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 //ICACHE_FLASH_ATTR
ESP_FirmwareUpdate(u8 *pu8FileData, u32 u32Offset, u32 u32DataLen)
{
    u32 u32LeftBuf, u32UsedLen;
    u32 ret;

    u32UsedLen = g_struOtaBuf.u16DateUsed;
    u32LeftBuf = PCT_OTA_BUF_LEN - u32UsedLen;
    ZC_Printf("OTA: used is %d, left is %d\n", u32UsedLen, u32LeftBuf);
    if (u32LeftBuf < u32DataLen)
    {
        /* copy as much as possible */
        memcpy(&g_struOtaBuf.u8OtaBuf[u32UsedLen],
                pu8FileData,
                u32LeftBuf);
        /* erase & write */
        ret = ESP_FlashEraseAddWrite();
        if (ZC_RET_OK != ret)
        {
            ZC_Printf("ESP_FlashEraseAddWrite error1\n");
            return ret;
        }
        /* copy */
        memcpy(&g_struOtaBuf.u8OtaBuf, pu8FileData + u32LeftBuf, u32DataLen - u32LeftBuf);
        g_struOtaBuf.u16DateUsed = u32DataLen - u32LeftBuf;
        return ZC_RET_OK;
    }
    else 
    {
        memcpy(&g_struOtaBuf.u8OtaBuf[u32UsedLen],
                pu8FileData,
                u32DataLen);
        g_struOtaBuf.u16DateUsed += u32DataLen;
        if (PCT_OTA_BUF_LEN == g_struOtaBuf.u16DateUsed)
        {
            ret = ESP_FlashEraseAddWrite();
            if (ZC_RET_OK != ret)
            {
                ZC_Printf("ESP_FlashEraseAddWrite error2\n");
                return ret;
            }
        }
    }
    
    return ZC_RET_OK;
}
/*************************************************
* Function: ESP_FlashEraseAddWrite
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 ESP_FlashEraseAddWrite(void)
{
    SpiFlashOpResult ret;
    ret = spi_flash_erase_sector(g_struProtocolController.u32OtaSectorNum);
    system_soft_wdt_feed();
    if (SPI_FLASH_RESULT_OK == ret)
    {
        system_soft_wdt_feed();
        ret = spi_flash_write(g_struProtocolController.u32OtaSectorNum * SECTOR_SIZE,
                                (uint32 *)g_struOtaBuf.u8OtaBuf,
                                SECTOR_SIZE);
        if (SPI_FLASH_RESULT_OK == ret)
        {
            ZC_Printf("OTA Write OK, sector num is 0x%x\n", g_struProtocolController.u32OtaSectorNum);
            g_struProtocolController.u32OtaSectorNum++;
            g_struOtaBuf.u16DateUsed = 0;
        }
        else
        {
            return ZC_RET_ERROR; 
        }
    }
    else 
    {
        return ZC_RET_ERROR;
    }
    return ZC_RET_OK;
}
/*************************************************
* Function: ESP_SendDataToMoudle
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 //ICACHE_FLASH_ATTR
ESP_SendDataToMoudle(u8 *pu8Data, u16 u16DataLen)
{
    AC_RecvMessage((ZC_MessageHead *)pu8Data);
    return ZC_RET_OK;
}
/*************************************************
* Function: ESP_Rest
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_Rest(void)
{
#if 0
    os_printf("ESP_Rest\n");
    g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig = 0;
    g_struZcConfigDb.struSwitchInfo.u32SecSwitch = 1;
    ESP_WriteDataToFlash((u8 *)&g_struZcConfigDb, sizeof(ZC_ConfigDB));
#endif
    g_struProtocolController.u8SmntFlag = SMART_CONFIG_STATE;

    xTaskCreate(smartconfig_task, "smartconfig_task", 256, NULL, 2, NULL);
}
/*************************************************
* Function: ESP_SendTcpData
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ESP_SendTcpData(u32 u32Fd, u8 *pu8Data, u16 u16DataLen, ZC_SendParam *pstruParam)
{
    send(u32Fd, pu8Data, u16DataLen, 0);
}
/*************************************************
* Function: HF_SendUdpData
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ESP_SendUdpData(u32 u32Fd, u8 *pu8Data, u16 u16DataLen, ZC_SendParam *pstruParam)
{
    sendto(u32Fd,(char*)pu8Data,u16DataLen,0,
        (struct sockaddr *)pstruParam->pu8AddrPara,
        sizeof(struct sockaddr_in)); 
}
/*************************************************
* Function: ESP_GetMac
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_GetMac(u8 *pu8Mac)
{
    unsigned char mac[6] = {0};
    unsigned char mac_string[ZC_SERVER_MAC_LEN] = {0};
    wifi_get_macaddr(STATION_IF, mac);
    ZC_HexToString(mac_string, mac, 6);
    memcpy(pu8Mac, mac_string, ZC_SERVER_MAC_LEN);
    ZCHEX_Printf(pu8Mac,ZC_SERVER_MAC_LEN);
}
/*************************************************
* Function: ESP_Reboot
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_Reboot(void)
{
    ZC_Printf("Reboot system !!!\n");
    system_restart();
}

/*************************************************
* Function: ESP_GotIp
* Description:
* Author: cxy
* Returns:
* Parameter:
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_GotIp(void)
{
	struct ip_info info;
	wifi_get_ip_info(0, &info);
	g_u32GloablIp = info.ip.addr;
}
/*************************************************
* Function: ESP_WakeUp
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_WakeUp()
{
    PCT_WakeUp();
    //ZC_StartClientListen();
}
/*************************************************
* Function: ESP_Sleep
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_Sleep()
{
    u32 u32Index;
    ZC_Printf("HF_Sleep\r\n");
    close(g_Bcfd);

    if (PCT_INVAILD_SOCKET != g_struProtocolController.struClientConnection.u32Socket)
    {
        close(g_struProtocolController.struClientConnection.u32Socket);
        g_struProtocolController.struClientConnection.u32Socket = PCT_INVAILD_SOCKET;
    }

    if (PCT_INVAILD_SOCKET != g_struProtocolController.struCloudConnection.u32Socket)
    {
        close(g_struProtocolController.struCloudConnection.u32Socket);
        g_struProtocolController.struCloudConnection.u32Socket = PCT_INVAILD_SOCKET;
    }
    
    for (u32Index = 0; u32Index < ZC_MAX_CLIENT_NUM; u32Index++)
    {
        if (0 == g_struClientInfo.u32ClientVaildFlag[u32Index])
        {
            close(g_struClientInfo.u32ClientFd[u32Index]);
            g_struClientInfo.u32ClientFd[u32Index] = PCT_INVAILD_SOCKET;
        }
    }

    PCT_Sleep();
    
    g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
    g_struUartBuffer.u32RecvLen = 0;

    g_struProtocolController.u32RecvAccessFlag = 0;
}
/*************************************************
* Function: ESP_CreateTaskTimer
* Description:
* Author: cxy
* Returns:
* Parameter:
* History:
*************************************************/
void ESP_ChangeToNormalState(void)
{
    g_struProtocolController.u8SmntFlag = 0;
}
/*************************************************
* Function: ESP_GetRandTime
* Description:
* Author: cxy
* Returns:
* Parameter:
* History:
*************************************************/
u32 ESP_GetRandTime(void)
{
    u32 u32Base, u32Timer;

    if (1 == g_struProtocolController.u32RecvAccessFlag)
    {
        g_struProtocolController.u32RecvAccessFlag = 0;
        u32Timer = PCT_TIMER_INTERVAL_RECONNECT;
        return u32Timer;
    }
    if (g_struProtocolController.struCloudConnection.u32ConnectionTimes > 20)
    {
        g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig = 0;
        g_struProtocolController.struCloudConnection.u32ConnectionTimes = 0;
    }
    u32Base = 5 * g_struProtocolController.struCloudConnection.u32ConnectionTimes;
    u32Timer = rand();
    u32Timer = (PCT_TIMER_INTERVAL_RECONNECT) * (u32Timer % 10 + 1 + u32Base);  
    return u32Timer;
}
/*************************************************
* Function: ESP_CloudRecvfunc
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
static void ESP_CloudRecvfunc(void) 
{
    s8 s8ret;
    s32 s32RecvLen=0; 
    fd_set fdread;
    u32 u32Index;
    u32 u32Len = 0; 
    u32 u32ActiveFlag = 0;
    struct sockaddr_in cliaddr;
    int connfd;
    extern u8 g_u8ClientStart;
    u32 u32MaxFd = 0;
    struct timeval timeout; 
    struct sockaddr_in addr;
    int tmp = 1;    

    ZC_StartClientListen();  /* 端口号9689，直连 */

    u32ActiveFlag = 0;
    
    timeout.tv_sec= 0; 
    timeout.tv_usec= 1000; 
    
    FD_ZERO(&fdread);

    FD_SET(g_Bcfd, &fdread);
    u32MaxFd = u32MaxFd > g_Bcfd ? u32MaxFd : g_Bcfd;
    
    /* 用于监听直连 */
    if (PCT_INVAILD_SOCKET != g_struProtocolController.struClientConnection.u32Socket)
    {
        FD_SET(g_struProtocolController.struClientConnection.u32Socket, &fdread);
        u32MaxFd = u32MaxFd > g_struProtocolController.struClientConnection.u32Socket ? u32MaxFd : g_struProtocolController.struClientConnection.u32Socket;
        u32ActiveFlag = 1;
    }
    /* 连接云端 */
    if ((g_struProtocolController.u8MainState >= PCT_STATE_WAIT_ACCESSRSP) 
        && (g_struProtocolController.u8MainState < PCT_STATE_DISCONNECT_CLOUD))
    {
        FD_SET(g_struProtocolController.struCloudConnection.u32Socket, &fdread);
        u32MaxFd = u32MaxFd > g_struProtocolController.struCloudConnection.u32Socket ? u32MaxFd : g_struProtocolController.struCloudConnection.u32Socket;
        u32ActiveFlag = 1;
    }

    for (u32Index = 0; u32Index < ZC_MAX_CLIENT_NUM; u32Index++)
    {
        if (0 == g_struClientInfo.u32ClientVaildFlag[u32Index])
        {
            FD_SET(g_struClientInfo.u32ClientFd[u32Index], &fdread);
            u32MaxFd = u32MaxFd > g_struClientInfo.u32ClientFd[u32Index] ? u32MaxFd : g_struClientInfo.u32ClientFd[u32Index];
            u32ActiveFlag = 1;            
        }
    }

    if (0 == u32ActiveFlag)
    {
        return;
    }
    
    s8ret = select(u32MaxFd + 1, &fdread, NULL, NULL, &timeout);
    if(s8ret <= 0)
    {
       return;
    }
    if ((g_struProtocolController.u8MainState >= PCT_STATE_WAIT_ACCESSRSP) 
        && (g_struProtocolController.u8MainState < PCT_STATE_DISCONNECT_CLOUD))
    {
        if (FD_ISSET(g_struProtocolController.struCloudConnection.u32Socket, &fdread))
        {
            s32RecvLen = recv(g_struProtocolController.struCloudConnection.u32Socket, g_u8recvbuffer, HF_MAX_SOCKET_LEN, 0); 
            
            if(s32RecvLen > 0) 
            {
                ZC_Printf("recv data len = %d\n", s32RecvLen);
                MSG_RecvDataFromCloud(g_u8recvbuffer, s32RecvLen);
            }
            else
            {
                ZC_Printf("recv error, len = %d\n",s32RecvLen);
                PCT_DisConnectCloud(&g_struProtocolController);
                
                g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
                g_struUartBuffer.u32RecvLen = 0;
            }
        }
        
    }

    for (u32Index = 0; u32Index < ZC_MAX_CLIENT_NUM; u32Index++)
    {
        if (0 == g_struClientInfo.u32ClientVaildFlag[u32Index])
        {
            /* g_struClientInfo.u32ClientFd 是tcp的新句柄 */
            if (FD_ISSET(g_struClientInfo.u32ClientFd[u32Index], &fdread))
            {
                s32RecvLen = recv(g_struClientInfo.u32ClientFd[u32Index], g_u8recvbuffer, HF_MAX_SOCKET_LEN, 0); 
                if (s32RecvLen > 0)
                {
                    ZC_RecvDataFromClient(g_struClientInfo.u32ClientFd[u32Index], g_u8recvbuffer, s32RecvLen);
                }
                else
                {   
                    ZC_ClientDisconnect(g_struClientInfo.u32ClientFd[u32Index]);
                    close(g_struClientInfo.u32ClientFd[u32Index]);
                }
                
            }
        }
        
    }

    if (PCT_INVAILD_SOCKET != g_struProtocolController.struClientConnection.u32Socket)
    {
        if (FD_ISSET(g_struProtocolController.struClientConnection.u32Socket, &fdread))
        {
            connfd = accept(g_struProtocolController.struClientConnection.u32Socket,(struct sockaddr *)&cliaddr, &u32Len);

            if (ZC_RET_ERROR == ZC_ClientConnect((u32)connfd))
            {
                close(connfd);
            }
            else
            {
                ZC_Printf("accept client = %d\n", connfd);
            }
        }
    }
    /* 局域网发现，收到别的设备发过来消息，同时发送自己的信息 */
    if (FD_ISSET(g_Bcfd, &fdread))
    {
        tmp = sizeof(addr); 
        s32RecvLen = recvfrom(g_Bcfd, g_u8BcSendBuffer, 100, 0, (struct sockaddr *)&addr, (socklen_t*)&tmp); 
        if(s32RecvLen > 0) 
        {
            ZC_SendClientQueryReq(g_u8BcSendBuffer, (u16)s32RecvLen);
        } 
    }
}
/*************************************************
* Function: ESP_ConnectToCloud
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 ESP_ConnectToCloud(PTC_Connection *pstruConnection)
{
    int fd; 
    struct sockaddr_in addr;
    struct ip_addr struIp;
    int retval;
    u16 port;
    int keepalive_enable = 1;
    memset((char*)&addr, 0, sizeof(addr));
    if (1 == g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig)
    {
        port = g_struZcConfigDb.struSwitchInfo.u16ServerPort;
        struIp.addr = htonl(g_struZcConfigDb.struSwitchInfo.u32ServerIp);
        retval = ZC_RET_OK;
    }
    else
    {
        port = ZC_CLOUD_PORT;
        retval = netconn_gethostbyname((const char *)g_struZcConfigDb.struCloudInfo.u8CloudAddr, &struIp);
    }

    if (ZC_RET_OK != retval)
    {
        return ZC_RET_ERROR;
    }

    ZC_Printf("connect ip = 0x%x!\n",struIp.addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = struIp.addr;
    fd = socket(AF_INET, SOCK_STREAM, 0);

    if(fd < 0)
    {
        return ZC_RET_ERROR;
    }
    if (ZC_CLOUD_PORT != port)
    {
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE,
               (const char *) &keepalive_enable, sizeof( keepalive_enable ));   
    }
    ZC_Printf("Start connecting...\n\r");
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr))< 0)
    {
        close(fd);
        if(g_struProtocolController.struCloudConnection.u32ConnectionTimes++>20)
        {
           g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig = 0;
        }

        return ZC_RET_ERROR;
    }
    g_struProtocolController.struCloudConnection.u32ConnectionTimes = 0;

    ZC_Printf("connect ok!\n");
    g_struProtocolController.struCloudConnection.u32Socket = fd;

    ZC_Rand(g_struProtocolController.RandMsg);
    return ZC_RET_OK;
}
/*************************************************
* Function: ESP_ListenClient
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 ESP_ListenClient(PTC_Connection *pstruConnection)
{
    int fd; 
    struct sockaddr_in servaddr;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd<0)
        return ZC_RET_ERROR;

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
    servaddr.sin_port = htons(pstruConnection->u16Port);
    if(bind(fd,(struct sockaddr *)&servaddr,sizeof(servaddr))<0)
    {
        close(fd);
        return ZC_RET_ERROR;
    }
    
    if (listen(fd, TCP_DEFAULT_LISTEN_BACKLOG)< 0)
    {
        close(fd);
        return ZC_RET_ERROR;
    }

    ZC_Printf("Tcp Listen Port = %d\n", pstruConnection->u16Port);
    g_struProtocolController.struClientConnection.u32Socket = fd;

    return ZC_RET_OK;
}

/*************************************************
* Function: ESP_BcInit
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ESP_BcInit(void)
{
    int tmp=1;
    struct sockaddr_in addr; 

    addr.sin_family = AF_INET; 
    addr.sin_port = htons(ZC_MOUDLE_PORT); 
    addr.sin_addr.s_addr=htonl(INADDR_ANY);

    g_Bcfd = socket(AF_INET, SOCK_DGRAM, 0); 

    tmp=1; 
    setsockopt(g_Bcfd, SOL_SOCKET,SO_BROADCAST,&tmp,sizeof(tmp)); 

    //hfnet_set_udp_broadcast_port_valid(ZC_MOUDLE_PORT, ZC_MOUDLE_PORT + 1);

    bind(g_Bcfd, (struct sockaddr*)&addr, sizeof(addr)); 
    g_struProtocolController.u16SendBcNum = 0;

    memset((char*)&struRemoteAddr,0,sizeof(struRemoteAddr));
    struRemoteAddr.sin_family = AF_INET; 
    struRemoteAddr.sin_port = htons(ZC_MOUDLE_BROADCAST_PORT); 
    struRemoteAddr.sin_addr.s_addr=inet_addr("255.255.255.255"); 
    g_pu8RemoteAddr = (u8*)&struRemoteAddr;
    g_u32BcSleepCount = 10;

    return;
}
/*************************************************
* Function: ESP_Cloudfunc
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
static void ESP_Cloudfunc(void* arg) 
{
    int fd;
    u32 u32Timer = 0;

    ESP_BcInit();

    while(1) 
    {    
    	if (TI_IS_IP_ACQUIRED(g_ulStatus))
        {
            ESP_GotIp();
            ESP_WakeUp();
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
        }
        if (TI_IS_DISCONNECTED(g_ulStatus))
        {
            ZC_Printf("ESP_Cloudfunc dis\n");
            ESP_Sleep();
            ESP_BcInit();
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_DISCONNECTED);
        }
        fd = g_struProtocolController.struCloudConnection.u32Socket;
        PCT_Run();
        ESP_CloudRecvfunc();
        Uart_RecvFromMcu();
        
        if (PCT_STATE_DISCONNECT_CLOUD == g_struProtocolController.u8MainState)
        {
            close(fd);
            if (0 == g_struProtocolController.struCloudConnection.u32ConnectionTimes)
            {
                u32Timer = 1000;
            }
            else
            {
                u32Timer = rand();
                u32Timer = (PCT_TIMER_INTERVAL_RECONNECT) * (u32Timer % 10 + 1);
            }
            PCT_ReconnectCloud(&g_struProtocolController, u32Timer);
            g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
            g_struUartBuffer.u32RecvLen = 0;
        }
        else
        {
            MSG_SendDataToCloud((u8*)&g_struProtocolController.struCloudConnection);
        }
        ZC_SendBc();
        sys_msleep(100);
    } 
}

/*************************************************
* Function: UARTRx_Buf_Init
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
UARTRx_Buf_Init(UARTStruct *qp, u8 *rxbuf, u16 len)
{
    PKT_DESC     *rx_desc = &(qp->Rx_desc);
    BUFFER_INFO  *rx_ring = &(qp->Rx_Buffer);
    
    rx_desc->pkt_num = 0;
    rx_desc->cur_num = 0;
    rx_desc->cur_type = PKT_UNKNOWN;
    Buf_init(rx_ring,(rxbuf),(u16)len);
}
/*************************************************
* Function: UartInit
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
UartInit(void)
{
    UARTRx_Buf_Init((UARTStruct*)(&UART0Port),(u8 *)(UART0RxBuf), UART0RX_RING_LEN);                             
    return;
}
/*************************************************
* Function: Uart_RecvFromMcu
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
Uart_RecvFromMcu(void)
{
    PKT_FIFO     *infor;
    PKT_DESC     *rx_desc = &(UART0Port.Rx_desc);
    BUFFER_INFO  *rx_ring = &(UART0Port.Rx_Buffer); 
    
    PKT_TYPE rxpkt_type;
    u16   rxpkt_len;
    u16 i;

    while (rx_desc->pkt_num)
    {
        //simulate FIFO,1st in,1st out
        infor = &(rx_desc->infor[0]);
        rxpkt_type = infor->pkt_type;
        rxpkt_len  = infor->pkt_len;
        
        memset(pCmdWifiBuf, 0, UART0RX_RING_LEN);
        
        //copy from uart rx ring
        for(i = 0; i < rxpkt_len; i++)       //O(n)
        {
            Buf_Pop(rx_ring,pCmdWifiBuf[i]);
            //ZC_Printf("Buf_Pop=%x \n",pCmdWifiBuf[i]);
        }
        //reset value
        infor->pkt_type = PKT_UNKNOWN;
        infor->pkt_len = 0;
        
        //shift FIFO
        for (i = 0; i < (rx_desc->pkt_num) - 1; i++)  //O(n)
        {
            rx_desc->infor[i]= rx_desc->infor[i+1];
        }  
        rx_desc->pkt_num--;
        
        //handle previous packet
        switch (rxpkt_type)
        {
            case PKT_PUREDATA:
                AC_UartProcess(pCmdWifiBuf, rxpkt_len);
                break;
            default:
                break;
        }    
        
    }

}
/*************************************************
* Function: ESP_Init
* Description:
* Author: hx 
* Returns: 
* Parameter: 
* History:
*************************************************/
int //ICACHE_FLASH_ATTR
ESP_Init(void)
{
	char mac_buf[MAC_LEN];
    char mac_string[ZC_HS_DEVICE_ID_LEN];
    u32 u32BinAddr;
	g_u64Domain = ((((u64)((SUB_DOMAIN_ID & 0xff00) >> 8)) << 48) + (((u64)(SUB_DOMAIN_ID & 0xff)) << 56) + (((u64)MAJOR_DOMAIN_ID & 0xff) << 40) + ((((u64)MAJOR_DOMAIN_ID & 0xff00) >> 8) << 32)
	+ ((((u64)MAJOR_DOMAIN_ID & 0xff0000) >> 16) << 24)
	+ ((((u64)MAJOR_DOMAIN_ID & 0xff000000) >> 24) << 16)
	+ ((((u64)MAJOR_DOMAIN_ID & 0xff00000000) >> 32) << 8)
	+ ((((u64)MAJOR_DOMAIN_ID & 0xff0000000000) >> 40) << 0));

    printf("ESP Init\n");

    g_struHfAdapter.pfunConnectToCloud = ESP_ConnectToCloud;
    g_struHfAdapter.pfunListenClient = ESP_ListenClient;
    g_struHfAdapter.pfunSendTcpData = ESP_SendTcpData;   
    g_struHfAdapter.pfunUpdate = ESP_FirmwareUpdate;     
    g_struHfAdapter.pfunUpdateFinish = ESP_FirmwareUpdateFinish;
    g_struHfAdapter.pfunSendToMoudle = ESP_SendDataToMoudle;  
    g_struHfAdapter.pfunSetTimer = ESP_SetTimer;   
    g_struHfAdapter.pfunStopTimer = ESP_StopTimer;
    g_struHfAdapter.pfunRest = ESP_Rest;
    g_struHfAdapter.pfunWriteFlash = ESP_WriteDataToFlash;
    g_struHfAdapter.pfunReadFlash = ESP_ReadDataFromFlash;
    g_struHfAdapter.pfunSendUdpData = ESP_SendUdpData;   
    g_struHfAdapter.pfunGetMac = ESP_GetMac;
    g_struHfAdapter.pfunReboot = ESP_Reboot;
    g_struHfAdapter.pfunUartSend = ESP_UartSend;

    PCT_Init(&g_struHfAdapter);

    g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
    g_struUartBuffer.u32RecvLen = 0;
    //ESP_ReadDataFromFlash();
    ESP_BcInit();

    memset(g_u8DeviceId, '0', ZC_HS_DEVICE_ID_LEN);
    memset(mac_string, '\0', ZC_HS_DEVICE_ID_LEN);
    
    wifi_get_macaddr(STATION_IF, mac_buf);
    ZCHEX_Printf(mac_buf, MAC_LEN);
    ZC_HexToString(mac_string, mac_buf, MAC_LEN);
    memcpy(g_u8DeviceId, mac_string, MAC_LEN * 2);

    memcpy(g_struRegisterInfo.u8PrivateKey, g_u8ModuleKey, ZC_MODULE_KEY_LEN);
    memcpy(g_struRegisterInfo.u8DeviciId, g_u8DeviceId, ZC_HS_DEVICE_ID_LEN);
    memcpy(g_struRegisterInfo.u8DeviciId + ZC_HS_DEVICE_ID_LEN, &g_u64Domain, ZC_DOMAIN_LEN);
    memcpy(g_struRegisterInfo.u8EqVersion, g_u8EqVersion, ZC_EQVERSION_LEN);

    u32BinAddr = system_get_userbin_addr();
    if (USER1_BIN_ADDR == u32BinAddr)
    {
        g_struProtocolController.u32UserBinNum = 0x81;
    }
    else if (USER2_BIN_ADDR == u32BinAddr)
    {
        g_struProtocolController.u32UserBinNum = 0x1;
    }

	if(xTaskCreate(ESP_Cloudfunc, ((const char*)"HF_Cloudfunc"), 512, NULL, 2, NULL) != 1)
    {   
		printf("\n\r%s xTaskCreate(init_thread) failed\n", __FUNCTION__);
    }
    return 1;

}

/******************************* FILE END ***********************************/


