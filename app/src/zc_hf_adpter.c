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



#define  FLASH_ADDRESS            0x200000

#define  HF_SUCCESS            0

#define AcTaskPrio        1
#define AcTaskQueueLen    10

u8  g_u8EqVersion[]={0,0,0,0};      
u8  g_u8ModuleKey[ZC_MODULE_KEY_LEN] =DEFAULT_IOT_PRIVATE_KEY;
u64  g_u64Domain;
u8  g_u8DeviceId[ZC_HS_DEVICE_ID_LEN];

//u64 SUB_DOMAIN_ID	=	0xFFFFFFFFFFFFFFFF;//子域id
vu32 g_u32DomainMagicFlag = 0xFFFFFFFF;

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

u16 g_u16TcpMss;
u16 g_u16LocalPort;

u8 g_u8recvbuffer[HF_MAX_SOCKET_LEN];
ZC_UartBuffer g_struUartBuffer;
ESP_TimerInfo g_struEspTimer[ZC_TIMER_MAX_NUM];

u8  g_u8BcSendBuffer[100];
u8  g_u8JdRcvBuffer[256];
u32 g_u32BcSleepCount = 800;//800


LOCAL struct espconn tcp_client_conn;    /* 做server */
struct espconn tcp_server_conn;          /* 做client */

LOCAL esp_tcp esptcp;
LOCAL struct espconn user_udp_espconn;

LOCAL os_timer_t task_timer;

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
* Function: ESP_ReadDataFormFlash
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_ReadDataFormFlash(void)
{
    u32 u32MagicFlag = 0xFFFFFFFF;

    spi_flash_read(FLASH_ADDRESS, (unsigned int *)(&u32MagicFlag), 4);
    if (ZC_MAGIC_FLAG == u32MagicFlag)
    {   
    	spi_flash_read(FLASH_ADDRESS, (unsigned int  *)(&g_struZcConfigDb), sizeof(ZC_ConfigDB));
    }
    else
    {
        ZC_Printf("no para, use default\n");
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
    //ZC_Printf("ESP_timer_callback: 1\n");
    u8 u8TimeId = *(u8 *)timer_arg;
    //ZC_Printf("ESP_timer_callback: u8TimeId is %d\n", u8TimeId);
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
    os_printf("ESP_Rest\n");
    g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig = 0;
    g_struZcConfigDb.struSwitchInfo.u32SecSwitch = 1;
    ESP_WriteDataToFlash((u8 *)&g_struZcConfigDb, sizeof(ZC_ConfigDB));

    g_struProtocolController.u8SmntFlag = SMART_CONFIG_STATE;
	SmartLink();
}
/*************************************************
* Function: ESP_SendTcpData
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_SendTcpData(u32 u32Fd, u8 *pu8Data, u16 u16DataLen, ZC_SendParam *pstruParam)
{
    struct espconn *pesp_conn = &tcp_client_conn;

    remot_info *premot = NULL;
    uint8 count = 0;
    sint8 value = ESPCONN_OK;

    /* client/server */
    if (PCT_SERVER_TCP_SOCKET == u32Fd)
    {
        espconn_send(&tcp_server_conn, pu8Data, u16DataLen);
        ZC_Printf("Send to cloud dataLen is :%d\n",u16DataLen);
        g_struProtocolController.u32AckFlag = 1;
    }
    else if (PCT_CLIENT_TCP_SOCKET == u32Fd)
    {
        if (espconn_get_connection_info(pesp_conn,&premot,0) == ESPCONN_OK)
        {
            ZC_Printf("link_cnt is %u\n", pesp_conn->link_cnt);
            for (count = 0; count < pesp_conn->link_cnt; count ++)
            {
                 pesp_conn->proto.tcp->remote_port = premot[count].remote_port;
                 
                 pesp_conn->proto.tcp->remote_ip[0] = premot[count].remote_ip[0];
                 pesp_conn->proto.tcp->remote_ip[1] = premot[count].remote_ip[1];
                 pesp_conn->proto.tcp->remote_ip[2] = premot[count].remote_ip[2];
                 pesp_conn->proto.tcp->remote_ip[3] = premot[count].remote_ip[3];

                 espconn_send(pesp_conn, pu8Data, u16DataLen);
                 ZC_Printf("Send to app dataLen is :%d\n",u16DataLen);
            }
        }    
    }
}
/*************************************************
* Function: ESP_SendUdpData
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_SendUdpData(u32 u32Fd, u8 *pu8Data, u16 u16DataLen, ZC_SendParam *pstruParam)
{
    espconn_send(&user_udp_espconn, pu8Data, u16DataLen);
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
* Function: ESP_SendToCloudSuccess
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_SendToCloudSuccess(void *arg)
{
    g_struProtocolController.u32AckFlag = 2;   /* got ack */
    ZC_Printf("ESP_SendToCloudSuccess success!!! \n");
}
/*************************************************
* Function: ESP_DisconFromCloud
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_DisconFromCloud(void *arg)
{
    u32 u32Timer;
    struct espconn *pespconn = arg;
    
    if (SMART_CONFIG_STATE == g_struProtocolController.u8SmntFlag)
    {
        return;
    }

	ZC_Printf("ESP_DisconFromCloud !!!\n");

    u32Timer = ESP_GetRandTime();
    if (tcp_server_conn.proto.tcp->remote_port == pespconn->proto.tcp->remote_port)
    {
        (void)espconn_disconnect(&tcp_server_conn);
        os_delay_us(1000);
        
        PCT_ReconnectCloud(&g_struProtocolController, u32Timer);

    }
}
/*************************************************
* Function: ESP_DisconFromRedirect
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_DisconFromRedirect(void *arg)
{
	ZC_Printf("ESP_DisconFromRedirect !!!\n");
}
/*************************************************
* Function: ESP_RecvFromCloud
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_RecvFromCloud(void *arg, char *pusrdata, unsigned short length)
{
    //received some data from tcp connection    
    ZC_Printf("recv from cloud, len is %d\n", length);
    memcpy(g_u8recvbuffer, pusrdata, length);
    ZC_Printf("copy ok\n");
    MSG_RecvDataFromCloud(g_u8recvbuffer, length);
}
/*************************************************
* Function: ESP_ConnectedCloud
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_ConnectedCloud(void *arg)
{
    u32 u32Keeplive;
    struct espconn *pespconn = arg;
    espconn_regist_recvcb(pespconn, ESP_RecvFromCloud);
    espconn_regist_sentcb(pespconn, ESP_SendToCloudSuccess);
    if (ZC_CLOUD_PORT != pespconn->proto.tcp->remote_port)
    {
        espconn_set_opt(pespconn, ESPCONN_KEEPALIVE); // enable TCP keep alive
        //set keepalive: 35s = 20 + 5*3 
        u32Keeplive = 20; 
        espconn_set_keepalive(pespconn, ESPCONN_KEEPIDLE, &u32Keeplive); 
        u32Keeplive = 5; 
        espconn_set_keepalive(pespconn, ESPCONN_KEEPINTVL, &u32Keeplive); 
        u32Keeplive = 3; //try times 
        espconn_set_keepalive(pespconn, ESPCONN_KEEPCNT, &u32Keeplive);
        espconn_regist_disconcb(pespconn, ESP_DisconFromCloud);
    }
    else
    {
        espconn_regist_disconcb(pespconn, ESP_DisconFromRedirect);
    }

	g_struProtocolController.u8MainState = PCT_STATE_WAIT_ACCESS;
	g_struProtocolController.u8keyRecv = PCT_KEY_UNRECVED;

    g_struProtocolController.struCloudConnection.u32ConnectionTimes = 0;
    ZC_Printf("connect succeed !!! \r\n");
}
/*************************************************
* Function: ESP_ReconnectCloud
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_ReconnectCloud(void *arg, sint8 err)
{
    u32 u32Timer;
    ZC_Printf("user error code %d !!! \r\n",err);
    if (SMART_CONFIG_STATE == g_struProtocolController.u8SmntFlag)
    {
        return;
    }

    //if(g_struProtocolController.u8MainState == PCT_STATE_INIT)return;
    u32Timer = ESP_GetRandTime();
	PCT_ReconnectCloud(&g_struProtocolController, u32Timer);
	g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
	g_struUartBuffer.u32RecvLen = 0;
	g_struProtocolController.u8MainState = PCT_STATE_ACCESS_NET;
}
/*************************************************
* Function: ESP_DnsFoundHook
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_DnsFoundHook(const char *name, ip_addr_t *ipaddr, void *arg)
{
    struct espconn *pespconn = (struct espconn *)arg;
    sint8 temp;

    if (NULL == ipaddr) 
    {
        ZC_Printf("ESP_DnsFoundHook NULL \r\n");
        return;
    }

   //dns got ip
    ZC_Printf("ESP_DnsFoundHook %d.%d.%d.%d \r\n",
            *((uint8 *)&ipaddr->addr), *((uint8 *)&ipaddr->addr + 1),
            *((uint8 *)&ipaddr->addr + 2), *((uint8 *)&ipaddr->addr + 3));

    if (ipaddr->addr != 0)
    {
        memcpy(tcp_server_conn.proto.tcp->remote_ip, &ipaddr->addr, 4); // remote ip of tcp server which get by dns
        tcp_server_conn.proto.tcp->remote_port = ZC_CLOUD_PORT; // remote port of tcp server
        tcp_server_conn.proto.tcp->local_port = espconn_port(); //local port of ESP8266

        espconn_regist_connectcb(&tcp_server_conn, ESP_ConnectedCloud); // register connect callback
        espconn_regist_reconcb(&tcp_server_conn, ESP_ReconnectCloud); // register reconnect callback as error handler
        temp = espconn_connect(&tcp_server_conn);
        if(temp == ESPCONN_ISCONN)
        {
            ZC_Printf("ESP_DnsFoundHook: already connected\n ");
            g_struProtocolController.struCloudConnection.u32ConnectionTimes = 0;
            g_struProtocolController.u8MainState = PCT_STATE_WAIT_ACCESS;
            return;
        }
        else
        {
            g_struProtocolController.struCloudConnection.u32ConnectionTimes++;
        }
        ZC_Printf("tcp connect %d, %d.%d.%d.%d, %d,%d\n",temp,
                tcp_server_conn.proto.tcp->remote_ip[0],
                tcp_server_conn.proto.tcp->remote_ip[1],
                tcp_server_conn.proto.tcp->remote_ip[2],
                tcp_server_conn.proto.tcp->remote_ip[3],
                tcp_server_conn.proto.tcp->remote_port,
				tcp_server_conn.proto.tcp->local_port);

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
u32 //ICACHE_FLASH_ATTR
ESP_ConnectToCloud(PTC_Connection *pstruConnection)
{
    struct ip_addr struIp;
    ip_addr_t tcp_server_ip;
    int retval = 255;
    u16 port;
    
    if (1 == g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig)
    {
        port = g_struZcConfigDb.struSwitchInfo.u16ServerPort;
        struIp.addr = ZC_HTONL(g_struZcConfigDb.struSwitchInfo.u32ServerIp);
        retval = HF_SUCCESS;
    }
    else
    {
        port = ZC_CLOUD_PORT;
        retval = espconn_gethostbyname(&tcp_server_conn,
                                        (const char *)g_struZcConfigDb.struCloudInfo.u8CloudAddr,
                                         &tcp_server_ip, ESP_DnsFoundHook);

        g_struProtocolController.u8MainState = PCT_STATE_INIT;
        return ZC_RET_OK;
    }
    
    ZC_Printf("connect cloud_default ip %d.%d.%d.%d %d\r\n",
            *((uint8 *)&struIp.addr), *((uint8 *)&struIp.addr + 1),
            *((uint8 *)&struIp.addr + 2), *((uint8 *)&struIp.addr + 3), port);

    memcpy(tcp_server_conn.proto.tcp->remote_ip, &struIp.addr, 4); 
    
    tcp_server_conn.proto.tcp->remote_port = port;
    tcp_server_conn.proto.tcp->local_port = espconn_port(); //local port of ESP8266
    
    espconn_regist_connectcb(&tcp_server_conn, ESP_ConnectedCloud); // register connect callback
    espconn_regist_reconcb(&tcp_server_conn, ESP_ReconnectCloud); // register reconnect callback as error handler
    
    if (0 != espconn_connect(&tcp_server_conn))
    {
        ZC_Printf("Connect not ok\n");
        g_struProtocolController.u8MainState = PCT_STATE_ACCESS_NET;
        return;
    }
    g_struProtocolController.struCloudConnection.u32ConnectionTimes++;
    g_struProtocolController.u8MainState = PCT_STATE_INIT;
    ZC_Rand(g_struProtocolController.RandMsg);

    return ZC_RET_OK;

}
/*************************************************
* Function: ESP_SendToClient
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_SendToClient(void *arg)
{
    ZC_Printf("tcp sent cb \r\n");
}
/*************************************************
* Function: ESP_RecvFromClient
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_RecvFromClient(void *arg, char *pusrdata, unsigned short length)
{
    struct espconn *pespconn = arg;
    ZC_Printf("ZC_RecvDataFromClient\n");
//    ZCHEX_Printf(pusrdata, length);
    memcpy(g_u8recvbuffer, pusrdata, length);
    ZC_RecvDataFromClient(0, g_u8recvbuffer, length);   /* 第一个参数没意义，传0进去即可 */
}
/*************************************************
* Function: ESP_DisconCliet
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_DisconCliet(void *arg)
{
    ZC_Printf("tcp disconnect succeed !!! \r\n");
}
/*************************************************
* Function: ESP_ReconClient
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_ReconClient(void *arg, sint8 err)
{
    ZC_Printf("reconnect callback, error code %d !!! \r\n",err);
}
/*************************************************
* Function: ESP_ClientConnected
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_ClientConnected(void *arg)
{
    struct espconn *pesp_conn = arg;

    ZC_Printf("Client connected\n");

    espconn_regist_recvcb(&tcp_client_conn, ESP_RecvFromClient);
    espconn_regist_reconcb(&tcp_client_conn, ESP_ReconClient);
    espconn_regist_disconcb(&tcp_client_conn, ESP_DisconCliet);
    espconn_regist_sentcb(&tcp_client_conn, ESP_SendToClient);

    memcpy(tcp_client_conn.proto.tcp->remote_ip, pesp_conn->proto.tcp->remote_ip,4);
    tcp_client_conn.proto.tcp->remote_port = pesp_conn->proto.tcp->remote_port;

    ZC_Printf("tcp_server_listen ip %d.%d.%d.%d port %d localport %d!!! \r\n"
    		,pesp_conn->proto.tcp->remote_ip[0]
			,pesp_conn->proto.tcp->remote_ip[1]
			,pesp_conn->proto.tcp->remote_ip[2]
			,pesp_conn->proto.tcp->remote_ip[3]
			,pesp_conn->proto.tcp->remote_port
			,pesp_conn->proto.tcp->local_port);
}
/*************************************************
* Function: ESP_ListenClient
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 //ICACHE_FLASH_ATTR
ESP_ListenClient(PTC_Connection *pstruConnection)
{
    tcp_client_conn.proto.tcp->local_port = pstruConnection->u16Port;
    ZC_Printf("Tcp Listen Port = %d\n", pstruConnection->u16Port);
    espconn_regist_connectcb(&tcp_client_conn, ESP_ClientConnected);
    s8 ret = espconn_accept(&tcp_client_conn);
    ZC_Printf("espconn_accept [%d] !!! \r\n", ret);

    return ZC_RET_OK;
}
/*************************************************
* Function: ESP_RecvFromBroadcast
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_RecvFromBroadcast(void *arg, char *pusrdata, unsigned short length)
{
    //ZC_Printf("recv udp data: \n");
    //ZCHEX_Printf(pusrdata,length);
    ZC_SendClientQueryReq(pusrdata,length);
}
/*************************************************
* Function: ESP_SendBroadcast
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void //ICACHE_FLASH_ATTR
ESP_SendBroadcast(void *arg)
{
    //    struct espconn *pespconn = arg;
    //ZC_Printf("Send Bc ok\n");
}
/*************************************************
* Function: ESP_BcInit
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_BcInit(void)
{
    user_udp_espconn.type = ESPCONN_UDP;
    user_udp_espconn.proto.udp = (esp_udp *)malloc(sizeof(esp_udp));

    tcp_server_conn.type = ESPCONN_TCP;
    tcp_server_conn.proto.tcp = (esp_tcp *)malloc(sizeof(esp_tcp));
    g_struProtocolController.struCloudConnection.u32Socket = PCT_SERVER_TCP_SOCKET;

    tcp_client_conn.type = ESPCONN_TCP;
    //tcp_client_conn.state = ESPCONN_NONE;
    tcp_client_conn.proto.tcp = (esp_tcp *)malloc(sizeof(esp_tcp));
      
    g_struProtocolController.struClientConnection.u32Socket = PCT_CLIENT_TCP_SOCKET;
    tcp_client_conn.proto.tcp->local_port = ZC_SERVER_PORT;
    
    g_struProtocolController.u16SendBcNum = 0;
    g_struProtocolController.u8MainState = PCT_STATE_INIT;
    g_u32BcSleepCount = 10;

    return;
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
	memcpy(user_udp_espconn.proto.udp->local_ip, (u8*)&(info.ip.addr), 4);
	g_u32GloablIp = info.ip.addr;

}
/*************************************************
* Function: ESP_UdpBroadcast
* Description:
* Author: cxy
* Returns:
* Parameter:
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_UdpBroadcast(void)
{
    user_udp_espconn.proto.udp->remote_ip[3] = 255;
    user_udp_espconn.proto.udp->remote_ip[2] = 255;
    user_udp_espconn.proto.udp->remote_ip[1] = 255;
    user_udp_espconn.proto.udp->remote_ip[0] = 255;

    user_udp_espconn.proto.udp->local_port = ZC_MOUDLE_PORT;
    user_udp_espconn.proto.udp->remote_port = ZC_MOUDLE_BROADCAST_PORT;  // ESP8266 udp remote port

    espconn_regist_recvcb(&user_udp_espconn, ESP_RecvFromBroadcast); // 收到数据回调
    espconn_regist_sentcb(&user_udp_espconn, ESP_SendBroadcast); // 发送数据成功回调
    
    espconn_create(&user_udp_espconn);   // create udp
}
/*************************************************
* Function: ESP_Init
* Description: 
* Author: cxy 
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
    g_struHfAdapter.pfunSendUdpData = ESP_SendUdpData;   
    g_struHfAdapter.pfunGetMac = ESP_GetMac;
    g_struHfAdapter.pfunReboot = ESP_Reboot;
    g_struHfAdapter.pfunUartSend = ESP_UartSend;

    g_u16TcpMss = 1000;

    PCT_Init(&g_struHfAdapter);

    g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
    g_struUartBuffer.u32RecvLen = 0;
    ESP_ReadDataFormFlash();
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
    ESP_CreateTaskTimer();
    return 1;

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
    ZC_StartClientListen();
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
    PCT_Sleep();
    
    g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
    g_struUartBuffer.u32RecvLen = 0;

    g_struProtocolController.u32AckFlag = 0;
    g_struProtocolController.u32RecvAccessFlag = 0;
}
/*************************************************
* Function: ESP_Cloudfunc
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
#if 1
static void //ICACHE_FLASH_ATTR
ESP_Cloudfunc(void *arg)
{
    u32 u32Timer;
    //disarm timer first
    os_timer_disarm(&task_timer);

	if (TI_IS_IP_ACQUIRED(g_ulStatus))
    {
        ZC_Printf("ESP_Cloudfunc ip aquired\n");
        ESP_GotIp();
        ESP_WakeUp();
    	ESP_UdpBroadcast();
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
    }
    if (TI_IS_DISCONNECTED(g_ulStatus))
    {
        ZC_Printf("ESP_Cloudfunc dis\n");
        ESP_Sleep();
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_DISCONNECTED);
    }
    Uart_RecvFromMcu();
    PCT_Run();
    
    if (PCT_STATE_DISCONNECT_CLOUD == g_struProtocolController.u8MainState)
    {
        u32Timer =  ESP_GetRandTime();

        PCT_ReconnectCloud(&g_struProtocolController, u32Timer);
        g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
        g_struUartBuffer.u32RecvLen = 0;
    }
    else
    {
        MSG_SendDataToCloud((u8*)&g_struProtocolController.struCloudConnection);
    }
    ZC_SendBc();
    
    os_timer_setfn(&task_timer, (os_timer_func_t *)ESP_Cloudfunc, NULL);
    os_timer_arm(&task_timer, 100, 0);

}
#else
static void //ICACHE_FLASH_ATTR
ESP_Cloudfunc(void *arg)
{
    u32 u32Timer;

    while (1)
    {
        //ZC_Printf("ESP_Cloudfunc\n");
    	if (TI_IS_IP_ACQUIRED(g_ulStatus))
        {
            ZC_Printf("ESP_Cloudfunc ip aquired\n");
            ESP_GotIp();
            ESP_WakeUp();
        	ESP_UdpBroadcast();
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
        }
        if (TI_IS_DISCONNECTED(g_ulStatus))
        {
            ZC_Printf("ESP_Cloudfunc dis\n");
            ESP_Sleep();
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_DISCONNECTED);
        }
        Uart_RecvFromMcu();
        PCT_Run();
        
        if (PCT_STATE_DISCONNECT_CLOUD == g_struProtocolController.u8MainState)
        {
            u32Timer =  ESP_GetRandTime();

            PCT_ReconnectCloud(&g_struProtocolController, u32Timer);
            g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
            g_struUartBuffer.u32RecvLen = 0;
        }
        else
        {
            MSG_SendDataToCloud((u8*)&g_struProtocolController.struCloudConnection);
        }
        ZC_SendBc();
        os_delay_us(100);
    }
    vTaskDelete(NULL);
}
#endif
/*************************************************
* Function: ESP_CreateTaskTimer
* Description:
* Author: cxy
* Returns:
* Parameter:
* History:
*************************************************/
void ESP_CreateTaskTimer(void)
{
#if 1
    os_timer_disarm(&task_timer);
    os_timer_setfn(&task_timer, (os_timer_func_t *)ESP_Cloudfunc, NULL);
    os_timer_arm(&task_timer, 100, 0);
#else
    xTaskCreate( ESP_Cloudfunc, ( signed char * ) "ESP_Cloudfunc", 384, ( void * ) NULL, 2, NULL); 
#endif
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

/******************************* FILE END ***********************************/


