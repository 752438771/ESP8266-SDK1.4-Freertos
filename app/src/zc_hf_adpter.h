/**
******************************************************************************
* @file     zc_hf_adpter.h
* @authors  cxy
* @version  V1.0.0
* @date     10-Sep-2014
* @brief    HANDSHAKE
******************************************************************************
*/

#ifndef  __ZC_HF_ADPTER_H__ 
#define  __ZC_HF_ADPTER_H__

#include <zc_common.h>
#include <zc_protocol_controller.h>
#include <zc_module_interface.h>
#include "esp_timer.h"
#include "bmd.h"

#define NUM_DESCS                         30
#define STAND_HEADER                      0x5A
#define STAND_TAIL                        0x5B
#define HEADER_LEN                        1 
#define AC_PAYLOADLENOFFSET              3
#define UART0RX_RING_LEN                 1024

#define SMART_CONFIG_STATE               1

#define USER1_BIN_ADDR                   0x1000
#define USER2_BIN_ADDR                   0x81000

#define SECTOR_SIZE                      PCT_OTA_BUF_LEN

#define MAC_LEN                           (6)

typedef struct 
{
    os_timer_t timer;
    u8         u8Index;
}ESP_TimerInfo;


#define HF_MAX_SOCKET_LEN    (1000)

//#ifdef WIFI_ON_MCU
typedef struct 
{
    u8 u8CloudKey[36];
    u8 u8PrivateKey[112];
    u8 u8DeviciId[ZC_HS_DEVICE_ID_LEN + ZC_DOMAIN_LEN];
    u8 u8CloudAddr[20];
    u8 u8EqVersion[ZC_EQVERSION_LEN];
    u8 u8TokenKey[16];
}HF_StaInfo;
//#endif

typedef enum {
    PKT_UNKNOWN,
    PKT_ATCMD,
    PKT_PUREDATA,
    PKT_ZADATA,
    PKT_PRINTCMD,
    PKT_HR01DATA
}PKT_TYPE;

typedef struct {
    PKT_TYPE pkt_type;
    u16   pkt_len;
}PKT_FIFO;//packet infor is in sequence with index[0,num_desc-1] which mapping the sequence in rx

typedef struct {
    PKT_TYPE  cur_type;              //receiving packet:which packet type is receiving current? 
    u8     cur_num;               //receiving packet:current index of receiving packet
    u8     pkt_num;               //completely packets:packet number in ring buffer
    PKT_FIFO  infor[NUM_DESCS];      //completely packets:FIFO,packet infor in ring
}PKT_DESC; 

typedef struct
{
    BUFFER_INFO                    Rx_Buffer;  //receive buffer
    PKT_DESC                       Rx_desc;    //description       
    BUFFER_INFO                    Tx_Buffer;  //transmit buffer    
} UARTStruct;

typedef enum{
    STATUS_BIT_NWP_INIT = 0, // If this bit is set: Network Processor is                           
    STATUS_BIT_CONNECTION,   // If this bit is set: the device is connected to                          
    STATUS_BIT_IP_LEASED,    // If this bit is set: the device has leased IP to 
    STATUS_BIT_IP_AQUIRED,   // If this bit is set: the device has acquired an IP
    STATUS_BIT_SMARTCONFIG_START, // If this bit is set: the SmartConfiguration 
    STATUS_BIT_P2P_DEV_FOUND,    // If this bit is set: the device (P2P mode) 
    STATUS_BIT_P2P_REQ_RECEIVED, // If this bit is set: the device (P2P mode) 
    STATUS_BIT_CONNECTION_FAILED, // If this bit is set: the device(P2P mode)
    STATUS_BIT_PING_DONE,         // If this bit is set: the device has completed
    STATUS_BIT_DISCONNECTED
}e_StatusBits;

#define GET_STATUS_BIT(status_variable, bit)    (0 != (status_variable & (1<<(bit))))
#define TI_IS_CONNECTED(status_variable)         GET_STATUS_BIT(status_variable, 1)
#define TI_IS_IP_ACQUIRED(status_variable)       GET_STATUS_BIT(status_variable, 3)
#define TI_IS_DISCONNECTED(status_variable)      GET_STATUS_BIT(status_variable, STATUS_BIT_DISCONNECTED)

#define SET_STATUS_BIT(status_variable, bit)     status_variable |= (1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit)     status_variable &= ~(1<<(bit))


#ifdef __cplusplus
extern "C" {
#endif
int ESP_Init(void);
void ESP_WakeUp(void);
void ESP_Sleep(void);
void ESP_ReadDataFormFlash(void);
void ESP_WriteDataToFlash(u8 *pu8Data, u16 u16Len);
void ESP_GotIp(void);
void ESP_WakeUp(void);
void UARTRx_Buf_Init(UARTStruct *qp, u8 *rxbuf, u16 len);
void UartInit(void);
void Uart_RecvFromMcu(void);
void ESP_ChangeToNormalState(void);
u32 ESP_FlashEraseAddWrite(void);
u32 ESP_GetRandTime(void);


#ifdef __cplusplus
}
#endif
#endif
/******************************* FILE END ***********************************/

