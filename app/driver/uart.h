/*
 * File	: uart.h
 * This file is part of Espressif's AT+ command set program.
 * Copyright (C) 2013 - 2016, Espressif Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 3 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#if 0
#ifndef UART_APP_H
#define UART_APP_H

#include "uart_register.h"
#include "eagle_soc.h"
#include "c_types.h"

#define RX_BUFF_SIZE    256
#define TX_BUFF_SIZE    100
#define UART0   0
#define UART1   1

#define PHONE   1
#define CLOUD   0


#define recvTaskPrio        0
#define recvTaskQueueLen    64
#define procTaskPrio        1
#define procTaskQueueLen    1

#define SIG_SEND_TO_SERVER  0
#define SIG_SEND_TO_CLIENT  1
#define SIG_TEST            255

typedef enum {
    FIVE_BITS = 0x0,
    SIX_BITS = 0x1,
    SEVEN_BITS = 0x2,
    EIGHT_BITS = 0x3
} UartBitsNum4Char;

typedef enum {
    ONE_STOP_BIT             = 0,
    ONE_HALF_STOP_BIT        = BIT2,
    TWO_STOP_BIT             = BIT2
} UartStopBitsNum;

typedef enum {
    NONE_BITS = 0,
    ODD_BITS   = 0,
    EVEN_BITS = BIT4
} UartParityMode;

typedef enum {
    STICK_PARITY_DIS   = 0,
    STICK_PARITY_EN    = BIT3 | BIT5
} UartExistParity;

typedef enum {
    BIT_RATE_9600     = 9600,
    BIT_RATE_19200   = 19200,
    BIT_RATE_38400   = 38400,
    BIT_RATE_57600   = 57600,
    BIT_RATE_74880   = 74880,
    BIT_RATE_115200 = 115200,
    BIT_RATE_230400 = 230400,
    BIT_RATE_256000 = 256000,
    BIT_RATE_460800 = 460800,
    BIT_RATE_921600 = 921600
} UartBautRate;

typedef enum {
    NONE_CTRL,
    HARDWARE_CTRL,
    XON_XOFF_CTRL
} UartFlowCtrl;

typedef enum {
    EMPTY,
    UNDER_WRITE,
    WRITE_OVER
} RcvMsgBuffState;

typedef struct {
    uint32     RcvBuffSize;
    uint8     *pRcvMsgBuff;
    uint8     *pWritePos;
    uint8     *pReadPos;
    uint8      TrigLvl; //JLU: may need to pad
    RcvMsgBuffState  BuffState;
} RcvMsgBuff;

typedef struct {
    uint32   TrxBuffSize;
    uint8   *pTrxBuff;
} TrxMsgBuff;

typedef enum {
    BAUD_RATE_DET,
    WAIT_SYNC_FRM,
    SRCH_MSG_HEAD,
    RCV_MSG_BODY,
    RCV_ESC_CHAR,
} RcvMsgState;

typedef struct {
    UartBautRate 	     baut_rate;
    UartBitsNum4Char  data_bits;
    UartExistParity      exist_parity;
    UartParityMode 	    parity;    // chip size in byte
    UartStopBitsNum   stop_bits;
    UartFlowCtrl         flow_ctrl;
    RcvMsgBuff          rcv_buff;
    TrxMsgBuff           trx_buff;
    RcvMsgState        rcv_state;
    int                      received;
    int                      buff_uart_no;  //indicate which uart use tx/rx buffer
} UartDevice;


#define WIFI_PACK_BUFSIZE   100
#define WIFI_DATA_PACK_BUFSIZE   10
typedef struct
{
	uint8 IP[4];
	uint8 WifiData[WIFI_PACK_BUFSIZE];
	uint8 DataLen;
}WIFI_PACK_TYPE;
typedef struct
{
  WIFI_PACK_TYPE   buf[WIFI_DATA_PACK_BUFSIZE];
  uint8               in;
  uint8               out;
}WIFI_RX_BUFFER;

WIFI_RX_BUFFER   WifiRxPack;              //Ω” ’


void uart_init(UartBautRate uart0_br, UartBautRate uart1_br);
void uart0_sendStr(const char *str);
void uart1_tx_buffer(uint8 *buf, uint16 len);

#endif
#else

#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "uart_register.h"

#define ETS_UART_INTR_ENABLE()  _xt_isr_unmask(1 << ETS_UART_INUM)
#define ETS_UART_INTR_DISABLE() _xt_isr_mask(1 << ETS_UART_INUM)
#define UART_INTR_MASK          0x1ff
#define UART_LINE_INV_MASK      (0x3f<<19)

typedef enum {
    UART_WordLength_5b = 0x0,
    UART_WordLength_6b = 0x1,
    UART_WordLength_7b = 0x2,
    UART_WordLength_8b = 0x3
} UART_WordLength;

typedef enum {
    USART_StopBits_1   = 0x1,
    USART_StopBits_1_5 = 0x2,
    USART_StopBits_2   = 0x3,
} UART_StopBits;

typedef enum {
    UART0 = 0x0,
    UART1 = 0x1,
} UART_Port;

typedef enum {
    USART_Parity_None = 0x2,
    USART_Parity_Even = 0x0,
    USART_Parity_Odd  = 0x1
} UART_ParityMode;

typedef enum {
    PARITY_DIS = 0x0,
    PARITY_EN  = 0x2
} UartExistParity;

typedef enum {
    BIT_RATE_300     = 300,
    BIT_RATE_600     = 600,
    BIT_RATE_1200    = 1200,
    BIT_RATE_2400    = 2400,
    BIT_RATE_4800    = 4800,
    BIT_RATE_9600    = 9600,
    BIT_RATE_19200   = 19200,
    BIT_RATE_38400   = 38400,
    BIT_RATE_57600   = 57600,
    BIT_RATE_74880   = 74880,
    BIT_RATE_115200  = 115200,
    BIT_RATE_230400  = 230400,
    BIT_RATE_460800  = 460800,
    BIT_RATE_921600  = 921600,
    BIT_RATE_1843200 = 1843200,
    BIT_RATE_3686400 = 3686400,
} UART_BautRate; //you can add any rate you need in this range

typedef enum {
    USART_HardwareFlowControl_None    = 0x0,
    USART_HardwareFlowControl_RTS     = 0x1,
    USART_HardwareFlowControl_CTS     = 0x2,
    USART_HardwareFlowControl_CTS_RTS = 0x3
} UART_HwFlowCtrl;

typedef enum {
    UART_None_Inverse = 0x0,
    UART_Rxd_Inverse  = UART_RXD_INV,
    UART_CTS_Inverse  = UART_CTS_INV,
    UART_Txd_Inverse  = UART_TXD_INV,
    UART_RTS_Inverse  = UART_RTS_INV,
} UART_LineLevelInverse;

typedef struct {
    UART_BautRate   baud_rate;
    UART_WordLength data_bits;
    UART_ParityMode parity;    // chip size in byte
    UART_StopBits   stop_bits;
    UART_HwFlowCtrl flow_ctrl;
    uint8           UART_RxFlowThresh ;
    uint32          UART_InverseMask;
} UART_ConfigTypeDef;

typedef struct {
    uint32 UART_IntrEnMask;
    uint8  UART_RX_TimeOutIntrThresh;
    uint8  UART_TX_FifoEmptyIntrThresh;
    uint8  UART_RX_FifoFullIntrThresh;
} UART_IntrConfTypeDef;

//=======================================

/** \defgroup Driver_APIs Driver APIs
  * @brief Driver APIs
  */

/** @addtogroup Driver_APIs
  * @{
  */

/** \defgroup UART_Driver_APIs UART Driver APIs
  * @brief UART driver APIs
  */

/** @addtogroup UART_Driver_APIs
  * @{
  */

/**  
  * @brief   Wait uart tx fifo empty, do not use it if tx flow control enabled.
  * 
  * @param   UART_Port uart_no:UART0 or UART1
  *  
  * @return  null
  */
void UART_WaitTxFifoEmpty(UART_Port uart_no); //do not use if tx flow control enabled

/**  
  * @brief   Clear uart tx fifo and rx fifo.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  *  
  * @return  null
  */
void UART_ResetFifo(UART_Port uart_no);

/**  
  * @brief  Clear uart interrupt flags.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   uint32 clr_mask : To clear the interrupt bits
  *  
  * @return  null
  */
void UART_ClearIntrStatus(UART_Port uart_no, uint32 clr_mask);

/**  
  * @brief   Enable uart interrupts .
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   uint32 ena_mask : To enable the interrupt bits
  *  
  * @return  null
  */
void UART_SetIntrEna(UART_Port uart_no, uint32 ena_mask);

/**  
  * @brief   Register an application-specific interrupt handler for Uarts interrupts.
  * 
  * @param   void *fn : interrupt handler for Uart interrupts.
  * @param   void *arg : interrupt handler's arg.
  *  
  * @return  null
  */
void UART_intr_handler_register(void *fn, void *arg);

/**  
  * @brief   Config from which serial output printf function.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  *  
  * @return  null
  */
void UART_SetPrintPort(UART_Port uart_no);

/**  
  * @brief   Config Common parameters of serial ports.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   UART_ConfigTypeDef *pUARTConfig : parameters structure
  *  
  * @return  null
  */
void UART_ParamConfig(UART_Port uart_no,  UART_ConfigTypeDef *pUARTConfig);

/**  
  * @brief   Config types of uarts.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   UART_IntrConfTypeDef *pUARTIntrConf : parameters structure
  *  
  * @return  null
  */
void UART_IntrConfig(UART_Port uart_no,  UART_IntrConfTypeDef *pUARTIntrConf);

/**  
  * @brief   Config the length of the uart communication data bits.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   UART_WordLength len : the length of the uart communication data bits
  *  
  * @return  null
  */
void UART_SetWordLength(UART_Port uart_no, UART_WordLength len);

/**  
  * @brief   Config the length of the uart communication stop bits.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   UART_StopBits bit_num : the length uart communication stop bits
  *  
  * @return  null
  */
void UART_SetStopBits(UART_Port uart_no, UART_StopBits bit_num);

/**  
  * @brief   Configure whether to open the parity.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   UART_ParityMode Parity_mode : the enum of uart parity configuration
  *  
  * @return  null
  */
void UART_SetParity(UART_Port uart_no, UART_ParityMode Parity_mode) ;

/**  
  * @brief   Configure the Baud rate.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   uint32 baud_rate : the Baud rate
  *  
  * @return  null
  */
void UART_SetBaudrate(UART_Port uart_no, uint32 baud_rate);

/**  
  * @brief   Configure Hardware flow control.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   UART_HwFlowCtrl flow_ctrl : Hardware flow control mode
  * @param   uint8 rx_thresh : threshold of Hardware flow control
  *  
  * @return  null
  */
void UART_SetFlowCtrl(UART_Port uart_no, UART_HwFlowCtrl flow_ctrl, uint8 rx_thresh);

/**  
  * @brief   Configure trigging signal of uarts.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   UART_LineLevelInverse inverse_mask : Choose need to flip the IO
  *  
  * @return  null
  */
void UART_SetLineInverse(UART_Port uart_no, UART_LineLevelInverse inverse_mask) ;

/**  
  * @brief   An example illustrates how to configure the serial port.
  * 
  * @param   null
  *  
  * @return  null
  */
void uart_init_new(void);

/**
  * @}
  */

/**
  * @}
  */
void uart0_handler(unsigned char *buf, unsigned short len);


#ifdef __cplusplus
}
#endif

#endif

#endif

